# smart_watch_nrfsdk

magnetometer used for compass , mpu6050 accelero and gyro 

atan2(y, x) = angle of the vector (x, y) from the +X axis
It returns the angle in radians, in the range:

-π  to  +π
(-180° to +180°)

atan2(y, x) gives the correct compass angle in all quadrants, even when x=0.

Magnetic declination = the angle between
Magnetic North (where your compass points) and
True North (geographic North Pole).

TODO:
find x ,y , z axis 

mpu6050 integration , calibration of qmc and mpu , integrae fusion algorithm
