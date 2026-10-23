# Rover -> ground station, published at 5 Hz. Missing readings are NaN.
float64 altitude     # m
float64 temperature  # deg C
float64 pressure     # kPa
geometry_msgs/Vector3 acceleration  # m/s^2

bool pico_connected  # a line arrived from the Pico in the last second
bool enabled         # the Pico reports its outputs as enabled
string note          # latest Pico status note or error
