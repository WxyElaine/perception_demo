#include "perception_demo/cube_segmenter.h"
#include <ros/console.h>

#include "visualization_msgs/Marker.h"
#include "pcl/segmentation/extract_clusters.h"
#include <pcl/filters/extract_indices.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include <boost/lexical_cast.hpp>

namespace segmentation {

Segmenter::Segmenter(const ros::Publisher& filtered_cloud_pub, const ros::Publisher& colored_cloud_pub, 
                     const ros::Publisher& marker_pub, const ros::Publisher& cube_centroid_pub)
  : filtered_cloud_pub_(filtered_cloud_pub), colored_cloud_pub_(colored_cloud_pub), 
    marker_pub_(marker_pub), cube_centroid_pub_(cube_centroid_pub) {}


void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  SegmentPointCloud(msg);
}


void Segmenter::SegmentPointCloud(const sensor_msgs::PointCloud2& msg) {
  // parameters
  int reg_distance, reg_point_color, reg_region_color, min_cluster_size, max_cluster_size;
  ros::param::param("reg_distance", reg_distance, 10);
  ros::param::param("reg_point_color", reg_point_color, 6);
  ros::param::param("reg_region_color", reg_region_color, 5);
  ros::param::param("min_cluster_size", min_cluster_size, 60);
  ros::param::param("max_cluster_size", max_cluster_size, 10000);

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_unfiltered (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::fromROSMsg (msg, *cloud_unfiltered);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, *indices);

//   // PASS THROUGH FILTER
//   pcl::PassThrough<pcl::PointXYZRGB> pass;
//   pass.setInputCloud(cloud);
//   pass.setFilterFieldName("z");
//   pass.setFilterLimits(0.0, 2.5);
//   pcl::PointCloud <pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
//   pass.filter(*filtered_cloud);

//   // publish the filtered point cloud
//   sensor_msgs::PointCloud2 filtered_cloud_msg;
//   pcl::toROSMsg (*filtered_cloud, filtered_cloud_msg);
//   filtered_cloud_pub_.publish(filtered_cloud_msg);


  // COLOR SEGMENTATION
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  // reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (reg_distance);  // determine whether the point is in the neighboring or not
  reg.setPointColorThreshold (reg_point_color);
  reg.setRegionColorThreshold (reg_region_color);
  reg.setMinClusterSize (min_cluster_size);
  reg.setMaxClusterSize (max_cluster_size);
  
  std::vector <pcl::PointIndices> object_indices;
  reg.extract (object_indices);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  
  // convert to sensor message
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg (*colored_cloud, cloud_out);

  // transform point cloud
  cloud_out.header.frame_id = "head_camera_depth_optical_frame";

  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  try {
    tf_listener.waitForTransform("map", cloud_out.header.frame_id,
                                 ros::Time(0), ros::Duration(5.0));
    tf_listener.lookupTransform("map", cloud_out.header.frame_id,
                                ros::Time(0), transform);
  } catch (tf::LookupException& e) {
    ROS_ERROR("%s", e.what());
  } catch (tf::ExtrapolationException& e) {
    ROS_ERROR("%s", e.what());
  }

  sensor_msgs::PointCloud2 colored_cloud_transformed;
  pcl_ros::transformPointCloud("map", cloud_out, colored_cloud_transformed, tf_listener);

  colored_cloud_pub_.publish(colored_cloud_transformed);

  // publish the color-segmented cloud
  colored_cloud_pub_.publish(colored_cloud_transformed);

  // convert ROS message to pcl PointCloud type for labelling
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud_with_transformation (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::fromROSMsg (colored_cloud_transformed, *colored_cloud_with_transformation);
  // label the markers
  SegmentMarker(&object_indices, colored_cloud_with_transformation);
}

void Segmenter::SegmentMarker(std::vector <pcl::PointIndices>* object_indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  // parameters
  int size_min, size_max;
  ros::param::param("size_min", size_min, 3);
  ros::param::param("size_max", size_max, 10);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);

  for (size_t i = 0; i < object_indices->size(); ++i) {
    // reify indices into a point cloud of the object
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices->at(i);

    if (indices->indices.size() > size_min && indices->indices.size() < size_max) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      // fill in object_cloud using indices
      extract.setIndices(indices);
      extract.setNegative(false);
      extract.filter(*object_cloud);

      // find the center of the object
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid (*object_cloud, centroid);

      geometry_msgs::Pose pose;
      pose.position.x = centroid[0];
      pose.position.y = centroid[1];
      pose.position.z = centroid[2];
      pose.orientation.w = 1;

      // publish a marker
      visualization_msgs::Marker object_marker;
      object_marker.ns = "objects";
      object_marker.id = 0;
      object_marker.header.frame_id = "map";
      object_marker.type = visualization_msgs::Marker::SPHERE;
      object_marker.pose = pose;
      object_marker.scale.x = 0.05;
      object_marker.scale.y = 0.05;
      object_marker.scale.z = 0.05;
      object_marker.color.g = 1;
      object_marker.color.a = 1;
      marker_pub_.publish(object_marker);

      // publish the centroid
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time(0);
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose = pose;
      cube_centroid_pub_.publish(pose_stamped);

    }
  }
}
}  // namespace segmentation