// create a ros node to subscribe to the point cloud topic and filter it based on a distance threshold


// ros includes
#include <ros/ros.h>

// includes for msgs
#include <sensor_msgs/PointCloud2.h>

// pcl specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>


ros::Publisher  pub;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // create a container for the data
  sensor_msgs::PointCloud2 output;

  // perform the actual filtering
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*cloud_filtered);

  // convert the cloud to ROS message
  pcl::toROSMsg(*cloud_filtered, output);

  // publish the data
  pub.publish(output);
}





int main(int argc, char** argv)
{
  // initialize the node
  ros::init(argc, argv, "distance_filter_node");

  // create a node handle
  ros::NodeHandle nh;

  // create a subscriber object
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);

  // create a publisher object
  pub = nh.advertise<sensor_msgs::PointCloud2>("distance_filter", 1);

  // spin
  ros::spin();
}


