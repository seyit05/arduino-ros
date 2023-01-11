#include <ros.h>
#include <MPU9255.h>//include MPU9255 library
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

MPU9255 mpu;
ros::NodeHandle  nh;

sensor_msgs::Imu imu;
ros::Publisher imu_publisher("imu", &imu);
std_msgs::Float64 msg_pitch;
std_msgs::Float64 msg_roll;
std_msgs::Float64 msg_yaw;
ros::Publisher pub_pitch("pitch", &msg_pitch);
ros::Publisher pub_roll("roll", &msg_roll);
ros::Publisher pub_yaw("yaw", &msg_yaw);


char base_link[] = "/base_link";
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


  //roll = atan2(ay_F , az_F) * 57.3; // deg
  //pitch = atan2((- ax_F) , sqrt(ay_F * ay_F + az_F * az_F)) * 57.3; // deg
  
  mx = low_pass_filter(mx, process_magnetic_flux(mpu.mx,mpu.mx_sensitivity) - 9.81, 0.7);
  my = low_pass_filter(mx, process_magnetic_flux(mpu.my,mpu.my_sensitivity) - 24.725, 0.7);
  mz = low_pass_filter(mx, process_magnetic_flux(mpu.mz,mpu.mz_sensitivity) - -27.395, 0.7);

  float pitch = atan2(ax_F,az_F)*57.3;
  float roll = atan2(ay_F,az_F)*57.3;
  float yaw = atan2(-mx*cos(roll*0.0175) + mz*sin(roll*0.0175), my*cos(pitch*0.0175) + mx*sin(pitch*0.0175)*sin(roll*0.0175) + mz*sin(pitch*0.0175)*cos(roll*0.0175)) * 57.3;

  //mx_h = my*cos(pitch*0.0175) + mx*sin(pitch*0.0175)*sin(roll*0.0175) + mz*sin(pitch*0.0175)*cos(roll*0.0175);
  //my_h = mx*cos(roll*0.0175) + mz*sin(roll*0.0175);


/***********************************************/
  // Gyroscope

  
  gx_F = high_pass_filter(mpu.gx,gx_prev, gx_F, 0.5);
  gy_F = high_pass_filter(mpu.gy,gy_prev, gy_F, 0.5);
  gz_F = high_pass_filter(mpu.gz,gz_prev, gz_F, 0.5);

  gx_prev = mpu.gx;
  gy_prev = mpu.gy;
  gz_prev = mpu.gz;

 
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.0175 * 0.5);
  double sy = sin(yaw * 0.0175 * 0.5);
  double cr = cos(roll * 0.0175 * 0.5);
  double sr = sin(roll * 0.0175 * 0.5);
  double cp = cos(pitch * 0.0175 * 0.5);
  double sp = sin(pitch * 0.0175 * 0.5);

  double qw = cy * cr * cp + sy * sr * sp;
  double qx = cy * sr * cp - sy * cr * sp;
  double qy = cy * cr * sp + sy * sr * cp;
  double qz = sy * cr * cp - cy * sr * sp;

  msg_roll.data = (roll);
  msg_pitch.data = (pitch);
  msg_yaw.data = (yaw);
  
  imu.header.frame_id= "/base_link";
  imu.orientation.x = qx;
  imu.orientation.y = qy;
  imu.orientation.z = qz;
  imu.orientation.w = qw;
  //imu_msg.orientation_covariance = {0,0,0,0,0,0,0,0,0};
  
  imu.angular_velocity.x = gx_F * 0.0175;
  imu.angular_velocity.y = gy_F * 0.0175;
  imu.angular_velocity.z = gz_F * 0.0175;
  //imu_msg.angular_velocity_covariance = {0,0,0,0,0,0,0,0,0};

  imu.linear_acceleration.x = ax_F*9.81;
  imu.linear_acceleration.y = ay_F*9.81;
  imu.linear_acceleration.z = az_F*9.81;
  //imu_msg.linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};
   
  
  imu_publisher.publish(&imu);
  pub_roll.publish(&msg_roll);
  pub_pitch.publish(&msg_pitch);
  pub_yaw.publish(&msg_yaw);
  nh.spinOnce();


 delay(1);
 
  
}
