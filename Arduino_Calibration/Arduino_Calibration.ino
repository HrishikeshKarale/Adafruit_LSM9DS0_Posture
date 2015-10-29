//need Adafruit_LSM9DS0 sensorapi https://github.com/adafruit/Adafruit_LSM9DS0_Library/tree/master/examples
//need runningMedian  http://playground.arduino.cc/Main/RunningMedian
//I used Arduino UNO.

//Please contact me if you find wrong calibration method.

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include "RunningMedian.h"

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

float Offset_ax, Offset_ay, Offset_az;
float Offset_gx, Offset_gy, Offset_gz;
float Offset_mx, Offset_my, Offset_mz;

int MedianV = 20;

float Ax , Ay, Az;
float Gx , Gy, Gz;
float Mx , My, Mz;

float BAx = 0 , BAy = 0, BAz = 0;
float BGx = 0 , BGy = 0, BGz = 0;
float BMx = 0 , BMy = 0, BMz = 0;

int State = 0;

void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}


void setup(void)
{
  while (!Serial);  // wait for flora/leonardo

  Serial.begin(9600);


  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");

  /* Initialise the sensor */
  if (!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));


  configureSensor();
}

void loop(void)
{
  if (State == 0)
  {
    Serial.println("\nReading sensors for first time...");
    SetData();
    State++;
    delay(1000);
  }

  if (State == 1) {
    Serial.println("\nCalculating offsets...");
    SetOffset();
    State++;
    delay(1000);
  }

  if (State == 2)
  {
    SetData();
    Ax -= Offset_ax;
    Ay -= Offset_ay;
    Az -= Offset_az;

    Gx -= Offset_gx;
    Gy -= Offset_gy;
    Gz -= Offset_gz;

    Mx -= Offset_mx;
    My -= Offset_my;
    Mz -= Offset_mz;

    //define Sensor Error
    //Smoothing
    float deadZoneAcc = 0.07;
    if (abs(Ax - BAx) < deadZoneAcc) Ax = BAx;
    if (abs(Ay - BAy) < deadZoneAcc) Ay = BAy;
    if (abs(Az - BAz) < deadZoneAcc) Az = BAz;

   
    float deadZoneGyro = 0.7;
    if (abs(Gx - BGx) < deadZoneGyro) Gx = BGx;
    if (abs(Gy - BGy) < deadZoneGyro) Gy = BGy;
    if (abs(Gz - BGz) < deadZoneGyro) Gz = BGz;


    float deadZoneM = 0.7;
    if (abs(Mx - BMx) < deadZoneM) Mx = BMx;
    if (abs(My - BMy) < deadZoneM) My = BMy;
    if (abs(Mz - BMz) < deadZoneM) Mz = BMz;
    
    if (abs(Ax) < deadZoneAcc) Ax = 0;
    if (abs(Ay) < deadZoneAcc) Ay = 0;
    if (abs(Az) < deadZoneAcc) Az = 0;

    if (abs(Gx) < deadZoneGyro) Gx = 0;
    if (abs(Gy) < deadZoneGyro) Gy = 0;
    if (abs(Gz) < deadZoneGyro) Gz = 0;

    if (abs(Mx) < deadZoneM) Mx = 0;
    if (abs(My) < deadZoneM) My = 0;
    if (abs(Mz) < deadZoneM) Mz = 0;


    Serial.print("A:"); Serial.print(Ax); Serial.print(","); Serial.print(Ay); Serial.print(","); Serial.print(Az); Serial.print("_");
    Serial.print("M:"); Serial.print(Mx); Serial.print(","); Serial.print(My); Serial.print(","); Serial.print(Mz); Serial.print("_");
    Serial.print("G:"); Serial.print(Gx); Serial.print(","); Serial.print(Gy); Serial.print(","); Serial.println(Gz);
    BAx = Ax;
    BAy = Ay;
    BAz = Az;

    BGx = Gx;
    BGy = Gy;
    BGz = Gz;

    BMx = Gx;
    BMy = Gy;
    BMz = Gz;
  }
  
  //delay(250);
}


void SetOffset()
{
  SetData();

  Offset_ax = Ax - 0;
  Offset_ay = Ay - 0;
  Offset_az = Az - 0;

  Offset_gx = Gx - 0;
  Offset_gy = Gy - 0;
  Offset_gz = Gz - 0;

  Offset_mx = Mx - 0;
  Offset_my = My - 0;
  Offset_mz = Mz - 0;
}


void SetData()
{
  RunningMedian Rax = RunningMedian(MedianV), Ray  = RunningMedian(MedianV), Raz  = RunningMedian(MedianV);
  RunningMedian Rgx = RunningMedian(MedianV), Rgy  = RunningMedian(MedianV), Rgz  = RunningMedian(MedianV);
  RunningMedian Rmx = RunningMedian(MedianV), Rmy  = RunningMedian(MedianV), Rmz  = RunningMedian(MedianV);
  sensors_event_t accel, mag, gyro, temp;
  for (int i = 0; i < MedianV; i++)
  {
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    float gx = gyro.gyro.x;
    float gy = gyro.gyro.y;
    float gz = gyro.gyro.z;

    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;

    float mx = mag.magnetic.x;
    float my = mag.magnetic.y;
    float mz = mag.magnetic.z;

    Rgx.add(gx);
    Rgy.add(gy);
    Rgz.add(gz);

    Rax.add(ax);
    Ray.add(ay);
    Raz.add(az);

    Rmx.add(mx);
    Rmy.add(my);
    Rmz.add(mz);

    i++;
    delay(2); 
  }

  /*
  Gx = Rgx.getMedian();
  Gy = Rgy.getMedian();
  Gz = Rgz.getMedian();

  Mx = Rgx.getMedian();
  My = Rgy.getMedian();
  Mz = Rgz.getMedian();

  Ax = Rax.getMedian();
  Ay = Ray.getMedian();
  Az = Raz.getMedian();
  */

  //Exclusion of outlier
  Gx = Rgx.getAverage(3);
  Gy = Rgy.getAverage(3);
  Gz = Rgz.getAverage(3);

  Mx = Rgx.getAverage(3);
  My = Rgy.getAverage(3);
  Mz = Rgz.getAverage(3);

  Ax = Rax.getAverage(3);
  Ay = Ray.getAverage(3);
  Az = Raz.getAverage(3);

}







