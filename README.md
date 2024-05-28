# imu_madgwick_filter
madgwick filter arduino library based on the imu_tool madgwick code by CCNYRoboticsLab [link](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble/imu_filter_madgwick)

follow the example `imu_sample_template.ino` code to see how to use it.

</br>

> Note each steps in the example code (based on the comments)
> shows you the process of using the madgwick filter
> Follow the steps and use it based on your own IMU.

</br>

> If you are using the MPU6050 (a 6-axis IMU) for example, you can skip the magnetometer part

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