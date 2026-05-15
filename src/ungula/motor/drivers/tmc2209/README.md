# TMC2209

## Sensorless Homing Notes

To detect stall conditions, the MC2209 requires STEALTHCHOP enabled.

When SG_RESULT (StallGuard result) is less then SGTHRS (StallGuard Threshold) the DIAG pin is set high

TCOOLTHRS (velocity threshold)
Stallguard will only work above this speed.

Adjust control Stallguard sensitivity if you find a linear relationship between your speed needs and the stallguard values returned by the driver.

Increase the value of Stallguard to make it MORE sensitive.
