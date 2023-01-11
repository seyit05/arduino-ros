#include <MadgwickAHRS.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <LSM6.h>
#include <LIS3MDL.h>
 
ros::NodeHandle nh;
geometry_msgs::Vector3 msg_accel;
ros::Publisher pub_accel("accel", &msg_accel);
geometry_msgs::Vector3 msg_magnet;
ros::Publisher pub_magnet("magnet", &msg_magnet);
geometry_msgs::Vector3 msg_gyro;
ros::Publisher pub_gyro("gyro", &msg_gyro);
std_msgs::Float64 msg_pitch;
std_msgs::Float64 msg_roll;
ros::Publisher pub_pitch("pitch", &msg_pitch);
ros::Publisher pub_roll("roll", &msg_roll);


unsigned long pubTimer = 0;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
float ax, ay, az;
float gx, gy, gz;
unsigned long microsNow;
double roll, pitch;


LIS3MDL mag;
LSM6 imu;
Madgwick filter;

void setup()
{
 
    nh.initNode();
    nh.advertise(pub_accel);
    nh.advertise(pub_magnet);
    nh.advertise(pub_gyro);
    nh.advertise(pub_pitch);
    nh.advertise(pub_roll);
 
    // Wait until connected
    while (!nh.connected())
        nh.spinOnce();
    nh.loginfo("ROS startup complete");
 
    Wire.begin();
 
    
    if (!mag.init())
      {
          nh.logerror("Failed to autodetect compass type!");
        
      }
    mag.enableDefault();

  filter.begin(25);

  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
 
    if (!imu.init())
      {
        nh.logerror("Failed to autodetect gyro type!");
      }
    imu.enableDefault();

  filter.begin(25);

  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
 
    pubTimer = millis();
}
 
void loop()
{


    if (millis() > pubTimer)

    if (microsNow - microsPrevious >= microsPerReading) 
      {
      
    
      {
        mag.read();
        imu.read();
         



        msg_accel.x = (float)(imu.a.x)*0.061/1000.0;
        msg_accel.y = (float)(imu.a.y)*0.061/1000.0;
        msg_accel.z = (float)(imu.a.z)*0.061/1000.0;
       
        msg_magnet.x = mag.m.x;
        msg_magnet.y = mag.m.y;
        msg_magnet.z = mag.m.z;
        
 
        msg_gyro.x = (float)(imu.g.x)*0.00875*M_PI/180.0;      
        msg_gyro.y = (float)(imu.g.y)*0.00875*M_PI/180.0;
        msg_gyro.z = (float)(imu.g.z)*0.00875*M_PI/180.0;

        ax = convertRawAcceleration(imu.a.x);
        ay = convertRawAcceleration(imu.a.y);
        az = convertRawAcceleration(imu.a.z);
        gx = convertRawGyro(imu.g.x);
        gy = convertRawGyro(imu.g.y);
        gz = convertRawGyro(imu.g.z);

        filter.updateIMU(gx, gy, gz, ax, ay, az);

        convertAngle(roll, imu.a.y, imu.a.z);
        convertAngle(pitch, imu.a.x, imu.a.z);

        msg_roll.data = (roll*180.0/M_PI);
        msg_pitch.data = (pitch*180.0/M_PI);
        delay(100);
  
        pub_accel.publish(&msg_accel);
        pub_magnet.publish(&msg_magnet);
        pub_gyro.publish(&msg_gyro);
        pub_roll.publish(&msg_roll);
        pub_pitch.publish(&msg_pitch);


        microsPrevious = microsPrevious + microsPerReading;
        }  
        pubTimer = millis() + 10;  // wait at least 10 msecs between publishing
        }
 
        nh.spinOnce();
        }

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
void convertAngle(double& loc, int val1, int val2)
{
  loc = atan((double)val1 / (double)val2);
}
