#include "neobot_core/neobot_core.h"

NeoBotCore::NeoBotCore(ros::NodeHandle& nh) : brake_(false){
  cmd_sub_ = nh.subscribe("/cmd_vel", 1, &NeoBotCore::handleCmdCallback, this);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  
  nh.param<std::string>("chassis_port", port_, "/dev/ttyUSB0");
  nh.param("baud", baud_, 115200);

  stm32_serial_ = std::unique_ptr<serial::Serial>(
    new serial::Serial(port_, baud_, serial::Timeout::simpleTimeout(300)));
  ROS_INFO("new serial");
  
  if(stm32_serial_->isOpen()){
    ROS_INFO("serial port initialized");
    std::thread trd(&NeoBotCore::ReadSerial, this);
    trd.detach();
    ROS_INFO("init serial thread");
  } else {
    ROS_INFO("serial port initialized failed");
  }
}

NeoBotCore::~NeoBotCore(){
  stm32_serial_.reset();
}

void NeoBotCore::ReadSerial(){
  odom_union odom;
  while(ros::ok()){
    stm32_serial_->read(odom.raw_data, sizeof(odom.raw_data));
      //ROS_INFO("size is %d", sizeof(odom.raw_data));
      //ROS_INFO("header data is %x", odom.data.header);
    if (swap16(odom.data.header) == 0x55BB){
      //ROS_INFO("got data: %x", odom.data.header);
      // ROS_INFO("got linear vel: %f", odom.data.odom_linear_vel);
      // ROS_INFO("got angular vel: %f", odom.data.odom_angular_vel);
      // ROS_INFO("got theta: %f", odom.data.odom_theta);
      // ROS_INFO("got odom x: %f", odom.data.odom_x);
      // ROS_INFO("got odom y: %f", odom.data.odom_y);
      nav_msgs::Odometry odom_data;
      geometry_msgs::TransformStamped odom_tf;
      // odom_data.twist.twist.linear.x = (double)swap32(odom.data.odom_linear_vel);
      // odom_data.twist.twist.linear.y = (double)swap32(odom.data.odom_angular_vel);
      // odom_data.pose.pose.position.x = (double)swap32(odom.data.odom_x);
      // odom_data.pose.pose.position.y = (double)swap32(odom.data.odom_y);
      // odom_data.pose.pose.orientation = tf::createQuaternionMsgFromYaw((double)swap32(odom.data.odom_theta));
      odom_data.header.frame_id = "odom";
      odom_data.header.stamp = ros::Time::now();
      odom_data.child_frame_id = "base_footprint";
      odom_data.twist.twist.linear.x = (double)odom.data.odom_linear_vel;
      odom_data.twist.twist.angular.z = (double)odom.data.odom_angular_vel;
      odom_data.pose.pose.position.x = (double)odom.data.odom_x;
      odom_data.pose.pose.position.y = (double)odom.data.odom_y;
      odom_data.pose.pose.orientation = tf::createQuaternionMsgFromYaw((double)odom.data.odom_theta);
      odom_pub_.publish(odom_data);
      
      odom_tf.header.frame_id = "odom";
      odom_tf.header.stamp = ros::Time::now();
      odom_tf.child_frame_id = "base_footprint";
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.translation.x = (double)odom.data.odom_x;
      odom_tf.transform.translation.y = (double)odom.data.odom_y;
      odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw((double)odom.data.odom_theta);
      odom_tf_.sendTransform(odom_tf);
    }
  }
};

void NeoBotCore::WriteSerial(){};

void NeoBotCore::handleCmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
  cmd_union cmd;
  cmd.data.header = swap16(0x55AA);
  cmd.data.linear_vel = (float)cmd_vel->linear.x;
  cmd.data.angular_vel = (float)cmd_vel->angular.z;
  cmd.data.brake = brake_;
  cmd.data.crc8 = 0;
  // ROS_INFO("size is %d", sizeof(cmd.raw_data));
  // ROS_INFO("send linear vel: %f", cmd.data.linear_vel);
  // ROS_INFO("send angular vel: %f", cmd.data.angular_vel);
  stm32_serial_->write(cmd.raw_data, sizeof(cmd.raw_data));
}

uint16_t NeoBotCore::swap16(uint16_t x){
  return (((uint16_t)(x) & 0x00FF) << 8) |
         (((uint16_t)(x) & 0xFF00) >> 8);
}

uint32_t NeoBotCore::swap32(uint32_t x){
  return (((uint32_t)(x) & 0xFF000000) >> 24) |
         (((uint32_t)(x) & 0x00FF0000) >> 8) |
         (((uint32_t)(x) & 0x0000FF00) << 8) |
         (((uint32_t)(x) & 0x000000FF) << 24);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "neobot_core");
    ros::NodeHandle nh("~");
    NeoBotCore neobot_core(nh);
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}