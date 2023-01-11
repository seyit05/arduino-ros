#include <ros.h>
#include<MPU9255.h>//include MPU9255 library
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64.h>
#include<sensor_msgs/MagneticField.h>

MPU9255 mpu;
ros::NodeHandle  nh;
char base_link[] = "/base_link";
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
std_msgs::Float64 msg_pitch;
std_msgs::Float64 msg_roll;
ros::Publisher mag_publisher("mag", &mag_msg);
ros::Publisher imu_publisher("imu", &imu_msg);
ros::Publisher pub_pitch("pitch", &msg_pitch);
ros::Publisher pub_roll("roll", &msg_roll);
float ax_F,ay_F,az_F;
float gx_F,gx_prev,gy_F,gy_prev,gz_F,gz_prev;
float mx,my,mz;

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

void calibrate_mag();

void setup() {
  nh.initNode();
  nh.advertise(imu_publisher);
  nh.advertise(mag_publisher);
  nh.advertise(pub_pitch);
  nh.advertise(pub_roll);
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


  float roll = atan2(ay_F , az_F) * 57.3; // deg
  float pitch = atan2((- ax_F) , sqrt(ay_F * ay_F + az_F * az_F)) * 57.3; // deg

  //float pitch = atan2(ax_F,az_F)*57.3;
  //float roll = atan2(ay_F,az_F)*57.3;

  mx = low_pass_filter(mx, process_magnetic_flux(mpu.mx,mpu.mx_sensitivity) - 9.81, 0.7);
  my = low_pass_filter(mx, process_magnetic_flux(mpu.my,mpu.my_sensitivity) - 24.725, 0.7);
  mz = low_pass_filter(mx, process_magnetic_flux(mpu.mz,mpu.mz_sensitivity) - -27.395, 0.7);


  //mx_h = my*cos(pitch*0.0175) + mx*sin(pitch*0.0175)*sin(roll*0.0175) + mz*sin(pitch*0.0175)*cos(roll*0.0175);
  //my_h = mx*cos(roll*0.0175) + mz*sin(roll*0.0175);

  
  float yaw = atan2(-mx*cos(roll*0.0175) + mz*sin(roll*0.0175), my*cos(pitch*0.0175) + mx*sin(pitch*0.0175)*sin(roll*0.0175) + mz*sin(pitch*0.0175)*cos(roll*0.0175)) * 57.3;



/***********************************************/
  // Gyroscope

  
  gx_F = high_pass_filter(mpu.gx,gx_prev, gx_F, 0.5);
  gy_F = high_pass_filter(mpu.gy,gy_prev, gy_F, 0.5);
  gz_F = high_pass_filter(mpu.gz,gz_prev, gz_F, 0.5);

  gx_prev = mpu.gx;
  gy_prev = mpu.gy;
  gz_prev = mpu.gz;

  msg_roll.data = roll;
  msg_pitch.data = pitch;
/* 
  // Abbreviations for the various angular functions
//  double cy = cos(yaw * 0.0175 * 0.5);
//  double sy = sin(yaw * 0.0175 * 0.5);
  double cr = cos(roll * 0.0175 * 0.5);
  double sr = sin(roll * 0.0175 * 0.5);
  double cp = cos(pitch * 0.0175 * 0.5);
  double sp = sin(pitch * 0.0175 * 0.5);

  double qw = (cy * cr * cp) + (sy * sr * sp);
  double qx = (cy * sr * cp) - (sy * cr * sp);
  double qy = (cy * cr) * (sp + sy * sr * cp);
  double qz = (sy * cr * cp) - (cy * sr * sp);
  imu_msg.header.frame_id= "/base_link";
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;
  imu_msg.orientation.w = qw;
  //imu_msg.orientation_covariance = {0,0,0,0,0,0,0,0,0};*/
  imu_msg.header.frame_id= "/base_link";
  imu_msg.angular_velocity.x = gx_F * 0.0175;
  imu_msg.angular_velocity.y = gy_F * 0.0175;
  imu_msg.angular_velocity.z = gz_F * 0.0175;
  //imu_msg.angular_velocity_covariance = {0,0,0,0,0,0,0,0,0};

  imu_msg.linear_acceleration.x = mpu.ax*9.81;
  imu_msg.linear_acceleration.y = ay_F*9.81;
  imu_msg.linear_acceleration.z = az_F*9.81;
  //imu_msg.linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};
  
  mag_msg.magnetic_field.x = mx;
  mag_msg.magnetic_field.y = my;
  mag_msg.magnetic_field.z = mz;


  
  pub_roll.publish(&msg_roll);
  pub_pitch.publish(&msg_pitch);
  imu_publisher.publish(&imu_msg);
  mag_publisher.publish(&mag_msg);
  nh.spinOnce();

/*
  Serial.print(" Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Yaw: ");
  Serial.print(yaw);
  Serial.println();
 */
  delay(1);
}
