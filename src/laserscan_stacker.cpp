#include <ros/ros.h>
#include <pcl/point_types.h>
#include <tf/message_filter.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>

double factor;
int size, overlap;
ros::Publisher pub;
std::deque<sensor_msgs::PointCloud> v;

laser_geometry::LaserProjection projector;

tf::TransformListener* tfListener;

pointcloud_msgs::PointCloud2_Segments bufferToAccumulator(const sensor_msgs::LaserScan::ConstPtr scan_in){
  sensor_msgs::PointCloud2 accumulator;

  double middle_z = 0.0;
  double z_diff = 0.0;
  int last_valid_index = -1;
  int tmp_index = -1;

   for (unsigned j=0; j < v.size(); j++) {
    if (j > 0) {
      tmp_index = -1;
      if (v[j-1].points.size()) {
        tmp_index = j-1;
      }
      else if (last_valid_index >= 0) {
        tmp_index = last_valid_index;
      }
      if (tmp_index >= 0) {
        z_diff = std::max(0.0, v[tmp_index].points[0].z + factor); //ros::Duration(v[j].header.stamp - v[tmp_index].header.stamp).toSec() * factor);
        for (size_t i=0; i < v[j].points.size(); i++) {
            v[j].points[i].z = z_diff;
        }
        if (v[j].points.size()) {
          last_valid_index = j;
        }
      }
      if (j == v.size() / 2 and v[j].points.size() and v[j].points[0].z > middle_z) {
        middle_z = v[j].points[0].z;
      }
    }
    else {
      for (size_t i=0; i < v[j].points.size(); i++) {
        v[j].points[i].z = 0.f;
      }
    }

    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(v[j], pc2);
    pcl::concatenatePointCloud( accumulator, pc2, accumulator);
  }

  pointcloud_msgs::PointCloud2_Segments c;

  c.header.stamp = ros::Time::now(); 
  c.clusters.push_back(accumulator);
  c.factor = factor;
  c.overlap = overlap ;
  if (v.size()) { // this should not be needed, but bsts
    c.first_stamp = v[0].header.stamp;
  }
  c.header.frame_id = scan_in->header.frame_id;
  //c.num_scans = num_scans;
  c.angle_min = scan_in->angle_min;
  c.angle_max = scan_in->angle_max;
  c.angle_increment = scan_in->angle_increment;
  c.range_min = scan_in->range_min;
  c.range_max = scan_in->range_max;
  c.scan_time = scan_in->scan_time;
  c.rec_time = scan_in->header.stamp;
  c.middle_z = middle_z;
  c.idForTracking = -1;

  return c;
}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr scan_in){
  sensor_msgs::PointCloud2 cloud;
  try{
    projector.transformLaserScanToPointCloud(scan_in->header.frame_id, *scan_in, cloud, *tfListener,
      laser_geometry::channel_option::Intensity | 
      laser_geometry::channel_option::Index | 
      laser_geometry::channel_option::Distance | 
      laser_geometry::channel_option::Timestamp | 
      laser_geometry::channel_option::Viewpoint);

    sensor_msgs::PointCloud pc1;
    sensor_msgs::convertPointCloud2ToPointCloud (cloud, pc1);

    v.push_back(pc1);

    if (v.size() == size){
      pub.publish(bufferToAccumulator(scan_in));
      // v.erase(v.begin(), v.begin() + overlap);
      v.erase(v.begin(), v.end());
    }
  }
  catch(...){}
}


int main(int argc, char** argv){

  ros::init(argc, argv, "laserscan_stacker");
  ros::NodeHandle n;

  std::string input_topic, out_topic;

  tfListener = new (tf::TransformListener);

  n.param("laserscan_stacker/out_topic", out_topic, std::string("laserscan_stacker/scans"));
  n.param("laserscan_stacker/input_topic", input_topic, std::string("/scan"));
  n.param("laserscan_stacker/size", size, 40);
  n.param("laserscan_stacker/factor", factor, 5.0);
  n.param("laserscan_stacker/overlap", overlap, 35);

  ros::Subscriber s = n.subscribe(input_topic, 1, scanCallback);
  pub = n.advertise<pointcloud_msgs::PointCloud2_Segments> (out_topic, 1);

  ros::spin();

  return 0;
}
