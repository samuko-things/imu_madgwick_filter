/*
 * MODIFIED BY OBIAGBA SAMUEL
 *
 * samuko-things
 */

#include <MatVectLab.h>
#include <imu_madgwick_filter.h>
#include "mpu9250.h"

float MicroTeslaToTesla(float mT)
{
  return mT * 1000000;
}

/* Mpu9250 object i2c */
bfs::Mpu9250 imu;

ImuMadgwickFilter madgwickFilter;

unsigned long serialCommTime, serialCommSampleTime = 10; // ms -> (1000/sampleTime) hz
unsigned long readImuTime, readImuSampleTime = 10;        // ms -> (1000/sampleTime) hz

void setup()
{
  /* Serial to display data */
  Serial.begin(115200);
  Serial.setTimeout(2);

  //---------------- START IMU IN I2C MODE -----------------------//
   /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  //----------------------------------------------------------------//

  //---------------- INITIALIZE IMU -----------------------//
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    while (1) {}
  }

  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    while (1) {}
  }
  //----------------------------------------------------------------//

  madgwickFilter.setAlgorithmGain(1.0);
  madgwickFilter.setDriftBiasGain(0.0);
  madgwickFilter.setWorldFrameId(0); // 0 - NWU, 1 - ENU, 2 - NED

  delay(2000);

  serialCommTime = millis();
  readImuTime = millis();

}

void loop()
{

  if ((millis() - readImuTime) >= readImuSampleTime)
  {

    if (imu.Read())
    {
      //------------READ ACC DATA AND CALIBRATE---------------//
      float ax = imu.accel_x_mps2();
      float ay = imu.accel_y_mps2();
      float az = imu.accel_z_mps2();

      // convert NED frame to NWU frame
      axRaw = ax * 1.00;
      ayRaw = ay * -1.00;
      azRaw = az * -1.00;

      axCal = axRaw - axOff;
      ayCal = ayRaw - ayOff;
      azCal = azRaw - azOff;
      //------------------------------------------------------//

      //-----------READ GYRO DATA AND CALIBRATE---------------//
      float gx = imu.gyro_x_radps();
      float gy = imu.gyro_y_radps();
      float gz = imu.gyro_z_radps();

      // convert NED frame to NWU frame
      gxRaw = gx * 1.00;
      gyRaw = gy * -1.00;
      gzRaw = gz * -1.00;

      gxCal = gxRaw - gxOff;
      gyCal = gyRaw - gyOff;
      gzCal = gzRaw - gzOff;
      //-----------------------------------------------------//

      //-----------READ MAG DATA AND CALIBRATE---------------//
      float mx = imu.mag_x_ut();
      float my = imu.mag_y_ut();
      float mz = imu.mag_z_ut();

      // convert NED frame to NWU frame
      mxRaw = mx * 1.00;
      myRaw = my * -1.00;
      mzRaw = mz * -1.00;

      // magCal = A_1*(magRaw - b) using the A matrix and b vector to remove the magnetic offsets
      mag_vect[0] = mxRaw;
      mag_vect[1] = myRaw;
      mag_vect[2] = mzRaw;

      vectOp.subtract(mag_vect, mag_vect, b_vect); // mag_vect = mag_vect - b_vect
      vectOp.transform(mag_vect, A_mat, mag_vect); // mag_vect = A_mat * mag_vect

      mxCal = mag_vect[0];
      myCal = mag_vect[1];
      mzCal = mag_vect[2];
      //---------------------------------------------------//

      //------------- APPLY MADWICK FILTER -----------------//
      float _ax = axCal;
      float _ay = ayCal;
      float _az = azCal;

      float _gx = gxCal;
      float _gy = gyCal;
      float _gz = gzCal;

      float _mx = MicroTeslaToTesla(mxCal);
      float _my = MicroTeslaToTesla(myCal);
      float _mz = MicroTeslaToTesla(mzCal);

      // update the filter, which computes orientation
      //gyro - rad/sec;   acc - m/sec;   mag - Tesla
      // madgwickFilter.madgwickAHRSupdateIMU(_gx, _gy, _gz, _ax, _ay, _az); // for 6 axis IMU
      madgwickFilter.madgwickAHRSupdate(_gx, _gy, _gz, _ax, _ay, _az, _mx, _my, _mz);

      float roll=0.00;
      float pitch=0.00;
      float yaw=0.00;

      float qw=0.00;
      float qx=0.00;
      float qy=0.00;
      float qz=0.00;

      madgwickFilter.getOrientationRPY(roll, pitch, yaw); // output in rad
      madgwickFilter.getOrientationQuat(qw, qx, qy, qz);
      //----------------------------------------------------//

      //------------- PRINT RESULT -----------------//
      Serial.print("Orientation: ");
      Serial.print(roll);
      Serial.print(" ");
      Serial.print(pitch);
      Serial.print(" ");
      Serial.println(yaw);
      //----------------------------------------------------//
      
    }

    readImuTime = millis(); 
  }
}