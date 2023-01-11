#include <ros.h>
#include <MPU9255.h>//include MPU9255 library
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <MadgwickAHRS.h>
MPU9255 mpu;
ros::NodeHandle  nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_publisher("imu", &imu_msg);
std_msgs::Float64 msg_pitch;
std_msgs::Float64 msg_roll;
std_msgs::Float64 msg_yaw;
ros::Publisher pub_pitch("pitch", &msg_pitch);
ros::Publisher pub_roll("roll", &msg_roll);
ros::Publisher pub_yaw("yaw", &msg_yaw);
geometry_msgs::Vector3 msg_accel;
ros::Publisher pub_accel("accel", &msg_accel);
geometry_msgs::Vector3 msg_magnet;
ros::Publisher pub_magnet("magnet", &msg_magnet);
geometry_msgs::Vector3 msg_gyro;
ros::Publisher pub_gyro("gyro", &msg_gyro);


char base_link[] = "/base_link";
float ax_F,ay_F,az_F;
float gx_F,gx_prev,gy_F,gy_prev,gz_F,gz_prev;
float mx,my,mz;
float roll, pitch, yaw;
float low_pass_filter(double val, double prev_filtered_val, double alpha){
  float filtered_val;
  filtered_val = (1-alpha)*val+alpha*prev_filtered_val;
  return filtered_val;     
}
  
float high_pass_filter(double val,double prev_val, double prev_filtered_val, double alpha){
  float filtered_val;
  filtered_val = (1-alpha)*prev_filtered_val+(1-alpha)*(val-prev_val);
  return filtered_val;
}

float process_magnetic_flux(int16_t input, double sensitivity)
{
  return (input*0.06*sensitivity)/0.6;
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

void calibrate_mag();

void setup() {
  nh.initNode();
  nh.advertise(pub_accel);
  nh.advertise(pub_magnet);
  nh.advertise(pub_gyro);
  nh.advertise(imu_publisher);
  nh.advertise(pub_pitch);
  nh.advertise(pub_roll);
  nh.advertise(pub_yaw);
  mpu.init();//initialize MPU9255 chip

  mpu.set_acc_scale(scale_2g);//set accelerometer scale
  mpu.set_gyro_scale(scale_250dps);//set gyroscope scale

  // get initial values for complementary filter
  mpu.read_acc();//get data from the accelerometer
  mpu.read_gyro();//get data from the gyroscope
  mpu.read_mag();//get data from the magnetometer

 
  ax_F = low_pass_filter(mpu.ax, mpu.ax, 0.8);
  ay_F = low_pass_filter(mpu.ay, mpu.ay, 0.8);
  az_F = low_pass_filter(mpu.az, mpu.az, 0.8);


  gx_F = high_pass_filter(mpu.gx,mpu.gx, mpu.gx, 0.5);
  gy_F = high_pass_filter(mpu.gy,mpu.gy, mpu.gy, 0.5);
  gz_F = high_pass_filter(mpu.gz,mpu.gz, mpu.gz, 0.5);


  mx = process_magnetic_flux(mpu.mx,mpu.mx_sensitivity) - 9.81;
  my = process_magnetic_flux(mpu.my,mpu.my_sensitivity) - 24.725;
  mz = process_magnetic_flux(mpu.mz,mpu.mz_sensitivity) - -27.395;

  
  
}

void loop() {
  
  mpu.read_acc();//get data from the accelerometer
  mpu.read_gyro();//get data from the gyroscope
  mpu.read_mag();//get data from the magnetometer


  // Accelorometer
  ax_F = low_pass_filter(mpu.ax, ax_F, 0.8);
  ay_F = low_pass_filter(mpu.ay, ay_F, 0.8);
  az_F = low_pass_filter(mpu.az, az_F, 0.8);


  roll = atan2(ay_F , az_F) * 57.3; // deg
  pitch = atan2((- ax_F) , sqrt(ay_F * ay_F + az_F * az_F)) * 57.3; // deg
  
  mx = low_pass_filter(mx, process_magnetic_flux(mpu.mx,mpu.mx_sensitivity) - 9.81, 0.7);
  my = low_pass_filter(mx, process_magnetic_flux(mpu.my,mpu.my_sensitivity) - 24.725, 0.7);
  mz = low_pass_filter(mx, process_magnetic_flux(mpu.mz,mpu.mz_sensitivity) - -27.395, 0.7);

  float pitch = atan2(ax_F,az_F)*57.3;
  float roll = atan2(ay_F,az_F)*57.3;
  float yaw = atan2(-mx*cos(roll*0.0175) + mz*sin(roll*0.0175), my*cos(pitch*0.0175) + mx*sin(pitch*0.0175)*sin(roll*0.0175) + mz*sin(pitch*0.0175)*cos(roll*0.0175)) * 57.3;

  //mx_h = my*cos(pitch*0.0175) + mx*sin(pitch*0.0175)*sin(roll*0.0175) + mz*sin(pitch*0.0175)*cos(roll*0.0175);
  //my_h = mx*cos(roll*0.0175) + mz*sin(roll*0.0175);



  
  gx_F = high_pass_filter(mpu.gx,gx_prev, gx_F, 0.5);
  gy_F = high_pass_filter(mpu.gy,gy_prev, gy_F, 0.5);
  gz_F = high_pass_filter(mpu.gz,gz_prev, gz_F, 0.5);

  gx_prev = mpu.gx;
  gy_prev = mpu.gy;
  gz_prev = mpu.gz;

  msg_roll.data=roll;
  msg_pitch.data=pitch;
  msg_yaw.data=yaw;
  
  
  msg_accel.x = (float)(ax_F)*0.061/1000.0;
  msg_accel.y = (float)(ay_F)*0.061/1000.0;
  msg_accel.z = (float)(az_F)*0.061/1000.0;
  

  msg_magnet.x = mx;
  msg_magnet.y = my;
  msg_magnet.z = mz;
        
 
  msg_gyro.x = (float)(gx_F)*0.00875*M_PI/180.0;      
  msg_gyro.y = (float)(gy_F)*0.00875*M_PI/180.0;
  msg_gyro.z = (float)(gz_F)*0.00875*M_PI/180.0;
  /*
 
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;
  imu_msg.orientation.w = qw;
  //imu_msg.orientation_covariance = {0,0,0,0,0,0,0,0,0};
    */
  
  imu_msg.angular_velocity.x = (float)(gx_F)*0.00875*M_PI/180.0;  
  imu_msg.angular_velocity.y = (float)(gy_F)*0.00875*M_PI/180.0;
  imu_msg.angular_velocity.z = (float)(gz_F)*0.00875*M_PI/180.0;
  //imu_msg.angular_velocity_covariance = {0,0,0,0,0,0,0,0,0};
  imu_msg.header.frame_id= "/base_link";
  imu_msg.linear_acceleration.x = (float)(ax_F)*0.061/1000.0;
  imu_msg.linear_acceleration.y = (float)(ay_F)*0.061/1000.0;
  imu_msg.linear_acceleration.z = (float)(az_F)*0.061/1000.0;
  //imu_msg.linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};
   

  imu_publisher.publish(&imu_msg);
  pub_roll.publish(&msg_roll);
  pub_pitch.publish(&msg_pitch);
  pub_accel.publish(&msg_accel);
  pub_magnet.publish(&msg_magnet);
  pub_gyro.publish(&msg_gyro);
  pub_yaw.publish(&msg_yaw);
  nh.spinOnce();

 delay(1);
 
  
}
