# Ground station -> rover, published at 20 Hz.
# This stream is also the heartbeat: if it stops, the rover stops all outputs.
bool enable      # false = stop all outputs
float32 axis1    # servo targets in degrees (the Pico clamps to its limits)
float32 axis2
float32 axis3a
float32 axis3b
float32 motor    # DC motor duty in percent, -100..100 (the Pico caps it)
