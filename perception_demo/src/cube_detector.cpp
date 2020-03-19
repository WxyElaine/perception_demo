#include <vector>
#include <iostream>
#include <sstream>

#include "perception_demo/cube_segmenter.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "perception/downsample.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cube_detector");
  ros::NodeHandle nh;

  // cloud used in color segmentation
  ros::Publisher filtered_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1, true);
  // color-segmented cloud
  ros::Publisher colored_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1, true); 
  // marker
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  // cube centroid
  ros::Publisher cube_centroid_pub =
      nh.advertise<geometry_msgs::PoseStamped>("cube_centroid", 10, true);


  segmentation::Segmenter segmenter(filtered_cloud_pub, colored_cloud_pub, marker_pub, cube_centroid_pub);
  
  ros::Publisher downsample_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/downsampled_cloud", 1, true);
  perception::Downsampler downsampler(downsample_pub);
  
  ros::Subscriber downsample_sub =
      nh.subscribe("/head_camera/depth_registered/points", 1, &perception::Downsampler::Callback, &downsampler);  // For real purpose
  
  ros::Subscriber segment_sub = 
      nh.subscribe("/downsampled_cloud", 1, &segmentation::Segmenter::Callback, &segmenter);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}