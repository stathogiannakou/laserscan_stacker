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


using namespace std;



std::string input_topic, out_topic, base_link_frame;

class LaserscanStacker{

public:
  int size, overlap, cnt, num_scans;
  float ang_min , ang_max , ang_incr , time_incr;
  float rng_min , rng_max , scan_tm;
  double factor;

  std::string frame_id;
  ros::Publisher pub;
  ros::NodeHandle node;
  ros::Time r_time;
  std::deque<sensor_msgs::PointCloud> v;

  laser_geometry::LaserProjection projector;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;

  tf::TransformListener tfListener;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_transform;

  LaserscanStacker(ros::NodeHandle n) :
    node(n),
    laser_sub(node, input_topic, 10),
    laser_transform(laser_sub, tfListener, base_link_frame, 10)
  {
    laser_transform.registerCallback(boost::bind(&LaserscanStacker::scanCallback, this, _1));
    laser_transform.setTolerance(ros::Duration(0.01));
    pub = node.advertise<pointcloud_msgs::PointCloud2_Segments> (out_topic, 5);
  }
  

  pointcloud_msgs::PointCloud2_Segments bufferToAccumulator(std::deque<sensor_msgs::PointCloud> v_, ros::Time rt){
    sensor_msgs::PointCloud2 accumulator;

    double middle_z = 0.0;
    double z_diff = 0.0;
    int last_valid_index = -1;
    int tmp_index = -1;

    for (unsigned j=0; j < v_.size(); j++) {
      if (j > 0) {
        tmp_index = -1;
        if (v_[j-1].points.size()) {
          tmp_index = j-1;
        }
        else if (last_valid_index >= 0) {
          tmp_index = last_valid_index;
        }
        if (tmp_index >= 0) {
          z_diff = std::max(0.0, v_[tmp_index].points[0].z + ros::Duration(v_[j].header.stamp - v_[tmp_index].header.stamp).toSec() * factor);
          for (size_t i=0; i < v_[j].points.size(); i++) {
              v_[j].points[i].z = z_diff;
          }
          if (v_[j].points.size()) {
            last_valid_index = j;
          }
        }
        if (j == v_.size() / 2 and v_[j].points.size() and v_[j].points[0].z > middle_z) {
          middle_z = v_[j].points[0].z;
        }
      }
      sensor_msgs::PointCloud2 pc2;
      sensor_msgs::convertPointCloudToPointCloud2(v_[j], pc2);
      pcl::concatenatePointCloud( accumulator, pc2, accumulator);
    }

    pointcloud_msgs::PointCloud2_Segments c;

    c.header.stamp = ros::Time::now(); 
    c.clusters.push_back(accumulator);
    c.factor = factor;
    c.overlap = overlap ;
    if (v_.size()) { // this should not be needed, but bsts
      c.first_stamp = v_[0].header.stamp;
    }
    c.header.frame_id = frame_id;
    c.num_scans = num_scans;
    c.angle_min = ang_min;
    c.angle_max = ang_max;
    c.angle_increment = ang_incr;
    c.range_min = rng_min;
    c.range_max = rng_max;
    c.scan_time = scan_tm;
    c.rec_time = rt;
    c.middle_z = middle_z;
    c.idForTracking = -1;

    return c;
  }


 void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){
    sensor_msgs::PointCloud2 cloud;
    try{
      projector.transformLaserScanToPointCloud(base_link_frame, *scan_in, cloud, tfListener, 1000.0);

      sensor_msgs::PointCloud pc1;
      sensor_msgs::convertPointCloud2ToPointCloud (cloud, pc1);
    
      v.push_back(pc1);

      r_time = scan_in->header.stamp;

      if (v.size() > size){
        pub.publish(bufferToAccumulator(v, r_time));
        v.erase(v.begin(), v.begin() + overlap);
      }

      frame_id = scan_in->header.frame_id;
      ang_min = scan_in->angle_min;
      ang_max = scan_in->angle_max ;
      ang_incr = scan_in->angle_increment;
      time_incr = scan_in->time_increment;
      rng_min = scan_in->range_min;
      rng_max = scan_in->range_max;
      scan_tm = scan_in->scan_time;
    }
    catch(...){}
  }

};

int main(int argc, char** argv){

  ros::init(argc, argv, "laserscan_stacker");
  ros::NodeHandle n;

  n.param("laserscan_stacker/out_topic", out_topic, std::string("laserscan_stacker/scans"));
  n.param("laserscan_stacker/input_topic", input_topic, std::string("/scan"));
  n.param("laserscan_stacker/base_link_frame", base_link_frame, std::string("base_link"));

  LaserscanStacker lstopc(n);

  n.param("laserscan_stacker/size", lstopc.size, 40);
  n.param("laserscan_stacker/factor", lstopc.factor, 5.0);
  n.param("laserscan_stacker/overlap", lstopc.overlap, 35);


  ros::spin();

  return 0;
}
