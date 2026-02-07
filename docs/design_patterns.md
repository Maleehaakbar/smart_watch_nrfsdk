1. To get the value of variables from function, instead of making them global, do pass by reference(passing output parameters as arguments)
    int read_sensor(const struct device *sensor)
    int read_sensor(int *x, int *y, int*z)

2. offload non-urgent work to system work queue

3. encapsulation by callback functions as driver APIs are static , and are not visible outside file.