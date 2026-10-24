// Rover arm Pico firmware: servos, TB6612 DC motor, BNO055 + BMP388, SD log.
//
// Serial protocol (USB, 115200, one line per message, '\n' terminated):
//   Host -> Pico
//     H              heartbeat (send every <1 s or outputs stop)
//     E              enable arm (needs a recent heartbeat)
//     X              stop all
//     S              request status
//     A,<joint>,<deg> servo target; joint = 0..3 (Axis1, Axis2, Axis3A, Axis3B)
//     M,<percent>    DC motor, -100..100; resend every <300 ms while held
//   Pico -> Host
//     D,ms,temp_c,press_kpa,ax,ay,az,alt_m        sensor row (empty = no reading)
//     T,dry_run,enabled,motor_pct,motor_max,imu,bmp,sd,sd_rows,
//       then target,min,max for each joint, then note       status (4 Hz + after commands)
//
// GPIO numbers below are NOT physical header pin numbers.

#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>

// ---- Configuration ----
constexpr bool DRY_RUN = true; // true: no servo/motor signals, sensors still run
constexpr uint32_t BAUDRATE = 115200;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 1000;
constexpr uint32_t MOTOR_TIMEOUT_MS = 300;
constexpr uint32_t SENSOR_INTERVAL_MS = 200;
constexpr uint32_t LOG_INTERVAL_MS = 1000;
constexpr uint32_t STATUS_INTERVAL_MS = 250;

struct Joint {
    const char *name;
    int pin;
    float min_deg, max_deg, home_deg;
    int min_us, max_us;
    bool reversed;
};

// Replace -1 with measured limits; a joint with -1 values is never driven.
// Axis3A/Axis3B may be mechanically coupled.
Joint joints[] = {
    {"Axis1",  15, -1, -1, -1, -1, -1, false},
    {"Axis2",  14, -1, -1, -1, -1, -1, false},
    {"Axis3A", 13, -70, 70, 0, -1, -1, false},
    {"Axis3B", 12, -1, -1, -1, -1, -1, false},
};
constexpr int JOINT_COUNT = sizeof(joints) / sizeof(joints[0]);

constexpr int MOTOR_IN1_PIN = 8;
constexpr int MOTOR_IN2_PIN = 7;
constexpr int MOTOR_PWM_PIN = 6;
constexpr int MOTOR_DIRECTION = 1;         // -1 to reverse
constexpr float MOTOR_MAX_PERCENT = 30.0f; // bench duty limit

constexpr int I2C_SDA_PIN = 4;
constexpr int I2C_SCL_PIN = 5;
constexpr uint8_t BNO055_ADDRESS = 0x28;
constexpr uint8_t BMP388_ADDRESS = 0x77;
constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25f;

constexpr int SD_SCK_PIN = 18;
constexpr int SD_MOSI_PIN = 19;
constexpr int SD_MISO_PIN = 16;
constexpr int SD_CS_PIN = 17;

// ---- State ----
Servo servos[JOINT_COUNT];
float targets[JOINT_COUNT];
Adafruit_BNO055 bno(55, BNO055_ADDRESS, &Wire);
Adafruit_BMP3XX bmp;
File logFile;

bool imu_ok = false, bmp_ok = false, sd_ok = false;
uint32_t sd_rows = 0;
bool enabled = false;
float motor_speed = 0;
uint32_t last_heartbeat = 0, last_motor = 0;
uint32_t last_sensor = 0, last_log = 0, last_status = 0;
const char *note = "Ready; outputs stopped";
float sensors[6] = {NAN, NAN, NAN, NAN, NAN, NAN}; // temp, press, ax, ay, az, alt

char line[64];
size_t line_len = 0;

// ---- Outputs ----
bool calibrated(int i) {
    const Joint &j = joints[i];
    return j.min_deg < j.max_deg && j.min_us >= 200 && j.min_us < j.max_us && j.max_us <= 3000;
}

// Joints without an angle range get a 0..180 preview range.
bool hasRange(int i) { return joints[i].min_deg < joints[i].max_deg; }
float lowDeg(int i) { return hasRange(i) ? joints[i].min_deg : 0; }
float highDeg(int i) { return hasRange(i) ? joints[i].max_deg : 180; }

int pulseFor(int i) {
    const Joint &j = joints[i];
    float f = (targets[i] - j.min_deg) / (j.max_deg - j.min_deg);
    if (j.reversed) f = 1.0f - f;
    return int(j.min_us + f * (j.max_us - j.min_us));
}

void applyMotor() {
    if (DRY_RUN) return;
    const float speed = enabled ? motor_speed * MOTOR_DIRECTION : 0;
    analogWrite(MOTOR_PWM_PIN, 0); // drop PWM before changing direction
    digitalWrite(MOTOR_IN1_PIN, speed > 0 ? HIGH : LOW);
    digitalWrite(MOTOR_IN2_PIN, speed < 0 ? HIGH : LOW);
    analogWrite(MOTOR_PWM_PIN, int(fabsf(speed) * 255 / 100));
}

void stopAll(const char *reason) {
    enabled = false;
    motor_speed = 0;
    note = reason;
    for (auto &s : servos) if (s.attached()) s.detach();
    applyMotor();
}

void enableArm() {
    if (millis() - last_heartbeat >= HEARTBEAT_TIMEOUT_MS || last_heartbeat == 0) {
        stopAll("Heartbeat missing");
        return;
    }
    enabled = true;
    note = "Arm enabled";
    if (DRY_RUN) return;
    for (int i = 0; i < JOINT_COUNT; ++i) {
        if (!calibrated(i) || servos[i].attached()) continue;
        // Pass the target pulse so attach doesn't jump to center first.
        servos[i].attach(joints[i].pin, joints[i].min_us, joints[i].max_us, pulseFor(i));
    }
}

void moveJoint(int i, float deg) {
    if (!enabled || i < 0 || i >= JOINT_COUNT || !isfinite(deg)) {
        stopAll("Joint command rejected");
        return;
    }
    targets[i] = constrain(deg, lowDeg(i), highDeg(i));
    note = "Joint target updated";
    if (!DRY_RUN && servos[i].attached()) servos[i].writeMicroseconds(pulseFor(i));
}

void setMotor(float speed) {
    if (!isfinite(speed) || speed < -100 || speed > 100) {
        stopAll("Invalid motor speed");
        return;
    }
    if (speed != 0 && !enabled) {
        stopAll("Motor command rejected");
        return;
    }
    motor_speed = constrain(speed, -MOTOR_MAX_PERCENT, MOTOR_MAX_PERCENT);
    last_motor = millis();
    note = speed == 0 ? "DC motor stopped" : "DC motor running";
    applyMotor();
}

// ---- Telemetry ----
void printField(float v) {
    Serial.print(',');
    if (isfinite(v)) Serial.print(v, 4);
}

void sendStatus() {
    last_status = millis();
    Serial.printf("T,%d,%d", DRY_RUN, enabled);
    printField(motor_speed);
    printField(MOTOR_MAX_PERCENT);
    Serial.printf(",%d,%d,%d,%lu", imu_ok, bmp_ok, sd_ok, (unsigned long)sd_rows);
    for (int i = 0; i < JOINT_COUNT; ++i) {
        printField(targets[i]);
        printField(lowDeg(i));
        printField(highDeg(i));
    }
    Serial.print(',');
    Serial.println(note);
}

void readSensors() {
    for (float &v : sensors) v = NAN;
    if (imu_ok) {
        const auto a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        sensors[2] = a.x();
        sensors[3] = a.y();
        sensors[4] = a.z();
    }
    if (bmp_ok && bmp.performReading()) {
        sensors[0] = bmp.temperature;
        sensors[1] = bmp.pressure / 1000.0f; // Pa -> kPa
        sensors[5] = 44330.0f * (1.0f - powf(bmp.pressure / (SEA_LEVEL_PRESSURE_HPA * 100.0f), 0.1903f));
    }
    Serial.print("D,");
    Serial.print(millis());
    for (float v : sensors) printField(v);
    Serial.println();
}

void logSensors() {
    if (!sd_ok) return;
    logFile.print(millis());
    for (float v : sensors) {
        logFile.print(',');
        if (isfinite(v)) logFile.print(v, 4);
    }
    logFile.println();
    logFile.flush();
    if (logFile.getWriteError()) {
        sd_ok = false;
        logFile.close();
        return;
    }
    ++sd_rows;
}

// ---- Commands ----
void handleLine(char *cmd) {
    char *args = strchr(cmd, ',');
    if (args) *args++ = '\0';
    if (strlen(cmd) != 1) {
        stopAll("Unknown command");
        return;
    }
    switch (cmd[0]) {
    case 'H':
        if (enabled && millis() - last_heartbeat >= HEARTBEAT_TIMEOUT_MS) stopAll("Late heartbeat; enable again");
        last_heartbeat = millis();
        return; // heartbeats don't trigger a status reply
    case 'E': enableArm(); break;
    case 'X': stopAll("Stopped by operator"); break;
    case 'S': break;
    case 'A': {
        char *deg = args ? strchr(args, ',') : nullptr;
        if (!deg) { stopAll("Bad servo command"); break; }
        *deg++ = '\0';
        moveJoint(atoi(args), atof(deg));
        break;
    }
    case 'M':
        if (!args) { stopAll("Bad motor command"); break; }
        setMotor(atof(args));
        break;
    default: stopAll("Unknown command");
    }
    sendStatus();
}

void readCommands() {
    while (Serial.available()) {
        const char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            line[line_len] = '\0';
            if (line_len > 0) handleLine(line);
            line_len = 0;
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = c;
        } else {
            line_len = 0;
            stopAll("Command too long");
        }
    }
}

// ---- Setup / loop ----
void setup() {
    Serial.begin(BAUDRATE);

    for (int i = 0; i < JOINT_COUNT; ++i)
        targets[i] = hasRange(i) ? joints[i].home_deg : 90;

    if (!DRY_RUN) {
        pinMode(MOTOR_IN1_PIN, OUTPUT);
        pinMode(MOTOR_IN2_PIN, OUTPUT);
        pinMode(MOTOR_PWM_PIN, OUTPUT);
        analogWriteFreq(20000);
        analogWriteRange(255);
        applyMotor();
    }

    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();
    imu_ok = bno.begin();
    bmp_ok = bmp.begin_I2C(BMP388_ADDRESS, &Wire);
    if (bmp_ok) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_25_HZ);
    }

    SPI.setSCK(SD_SCK_PIN);
    SPI.setTX(SD_MOSI_PIN);
    SPI.setRX(SD_MISO_PIN);
    if (SD.begin(SD_CS_PIN, SPI)) {
        char name[16];
        for (int n = 0; n < 10000 && !sd_ok; ++n) {
            snprintf(name, sizeof(name), "LOG%04d.CSV", n);
            if (SD.exists(name)) continue;
            logFile = SD.open(name, FILE_WRITE);
            sd_ok = bool(logFile);
            if (sd_ok) logFile.println("ms,temp_c,press_kpa,accel_x,accel_y,accel_z,alt_m");
            break;
        }
    }

    sendStatus();
}

void loop() {
    const uint32_t now = millis();

    if (enabled && now - last_heartbeat >= HEARTBEAT_TIMEOUT_MS) {
        stopAll("Heartbeat timed out");
        sendStatus();
    }
    if (motor_speed != 0 && now - last_motor >= MOTOR_TIMEOUT_MS) {
        motor_speed = 0;
        note = "DC motor command timed out";
        applyMotor();
        sendStatus();
    }

    readCommands();

    if (now - last_sensor >= SENSOR_INTERVAL_MS) {
        last_sensor = now;
        readSensors();
    }
    if (now - last_log >= LOG_INTERVAL_MS) {
        last_log = now;
        logSensors();
    }
    if (now - last_status >= STATUS_INTERVAL_MS) sendStatus();
}
