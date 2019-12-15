#include <ros/ros.h>
#include "loam_velodyne/TransformMaintenance.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "transformMaintenance");
  ros::NodeHandle node;
  // 句柄可以让你通过构造函数指定命名空间
  // 如  ros::NodeHandle nh("my_namespace");
  // 私有名字，使用“～”（私有命名空间）
  // 可以在一个私有命名空间中直接创建一个新的句柄
  ros::NodeHandle privateNode("~");

  loam::TransformMaintenance transMaintenance;

  if (transMaintenance.setup(node, privateNode)) {
    // initialization successful
    ros::spin();
  }

  return 0;
}
  