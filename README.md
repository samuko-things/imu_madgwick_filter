# imu_madgwick_filter
madgwick filter arduino library based on the imu_tool madgwick code by CCNYRoboticsLab [link](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble/imu_filter_madgwick)

## How to Use the Library
- Download download the library by clicking on the green Code button above (or clone it)

- Move the downloaded library file to your Arduino library folder
  > e.g on linux: ... home/Arduino/libraries/
  >
  > e.g on windows: ... Documents/Arduino/libraries/

- follow the example `imu_sample_template.ino` code to see how to use it.

> NOTE: you can just check how I used it with the MPU6050 (a 6-axis IMU) - [link](https://github.com/samuko-things/arduino_mpu6050)

## Things to NOTE before using the library
> Note each steps in the example code (based on the comments)
> shows you the process of using the madgwick filter
> Follow the steps and use it based on your own IMU.

</br>

> If you are using the MPU6050 (a 6-axis IMU) for example, you can skip the magnetometer part - [sample code](https://github.com/samuko-things/arduino_mpu6050)

</br>

> you just need to basically know how to read the accelerometer, gyroscope (and magnetometer) from your IMU

</br>

> you will also need to calibrate them and get a calibrated reading.

</br>

> you will need to convert the sensor readings to the right units before passing them through the madgwick filter.
>- convert accelerometer reading to m/s^2
>- convert gyroscope reading to rad/sec
>- convert magnetometer readings to Tesla

</br>

> Ensure you know the frame of reference you are working with and specify it during the filter setup.
>- whether NED or EWU or NWU frame of reference