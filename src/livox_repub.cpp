#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"

inline bool sys_is_little_endian(){
        int i = 1;
        return (bool)(*(char*)&i);
}

ros::Publisher pub_pcl_out1;

void LivoxMsgCbk1(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  sensor_msgs::PointCloud2 pcl_ros_msg;
  unsigned int pcd_size = livox_msg_in->point_num;
  pcl_ros_msg.width = pcd_size;
  pcl_ros_msg.height = 1;
  pcl_ros_msg.is_bigendian = !sys_is_little_endian();
  pcl_ros_msg.point_step = sizeof(float) * 6 + sizeof(uint16_t) * 1;
  pcl_ros_msg.row_step = pcl_ros_msg.point_step * pcd_size;
  pcl_ros_msg.is_dense = true;
  pcl_ros_msg.fields.resize(7);

  pcl_ros_msg.fields[0].name = "x";
  pcl_ros_msg.fields[0].offset = 0;
  pcl_ros_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  pcl_ros_msg.fields[0].count = 1;
  pcl_ros_msg.fields[1].name = "y";
  pcl_ros_msg.fields[1].offset = 4;
  pcl_ros_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  pcl_ros_msg.fields[1].count = 1;
  pcl_ros_msg.fields[2].name = "z";
  pcl_ros_msg.fields[2].offset = 8;
  pcl_ros_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  pcl_ros_msg.fields[2].count = 1;
  pcl_ros_msg.fields[3].name = "intensity";
  pcl_ros_msg.fields[3].offset = 12;
  pcl_ros_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  pcl_ros_msg.fields[3].count = 1;
  pcl_ros_msg.fields[4].name = "ring";
  pcl_ros_msg.fields[4].offset = 16;
  pcl_ros_msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  pcl_ros_msg.fields[4].count = 1;
  pcl_ros_msg.fields[5].name = "time";
  pcl_ros_msg.fields[5].offset = 18;
  pcl_ros_msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
  pcl_ros_msg.fields[5].count = 1;
  pcl_ros_msg.fields[6].name = "range";
  pcl_ros_msg.fields[6].offset = 22;
  pcl_ros_msg.fields[6].datatype = sensor_msgs::PointField::FLOAT32;
  pcl_ros_msg.fields[6].count = 1;

  pcl_ros_msg.data.resize(pcd_size * pcl_ros_msg.point_step, 0x00);
  uint8_t *ptr = pcl_ros_msg.data.data();
  for(unsigned int i = 0; i < pcd_size; ++i){
    *(reinterpret_cast<float*>(ptr +  0))  = livox_msg_in->points[i].x;
    *(reinterpret_cast<float*>(ptr +  4))  = livox_msg_in->points[i].y;
    *(reinterpret_cast<float*>(ptr +  8))  = livox_msg_in->points[i].z;
    *(reinterpret_cast<float*>(ptr +  12))  = (float)livox_msg_in->points[i].reflectivity;
    *(reinterpret_cast<uint16_t*>(ptr +  16))  = (uint16_t)livox_msg_in->points[i].line;
    *(reinterpret_cast<float*>(ptr +  18))  = (float)livox_msg_in->points[i].offset_time / (float)1e9;
    *(reinterpret_cast<float*>(ptr +  22))  = sqrt((livox_msg_in->points[i].x * livox_msg_in->points[i].x) + 
        (livox_msg_in->points[i].y * livox_msg_in->points[i].y) + 
        (livox_msg_in->points[i].z * livox_msg_in->points[i].z));

    ptr += pcl_ros_msg.point_step;
  }

  unsigned long timebase_ns = livox_msg_in->timebase;
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "/velodyne";
  pub_pcl_out1.publish(pcl_ros_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCbk1);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 100);

  ros::spin();
}
