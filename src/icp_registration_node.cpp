

// write a ros node to match a point cloud to a szene based on icp and pcl

#include <ros/ros.h>


#include <sensor_msgs/PointCloud2.h>



#include <pcl/registration/icp.h>



// create the callback function for the point cloud
void scene_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);

}



void model_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);

}


int main(int argc, char** argv)
{
    ros::NodeHandle nh;


    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>)  
    // Fill in the CloudIn data
    for (auto& point : *cloud_in)
    {
      point.x = 1024 * rand() / (RAND_MAX + 1.0f);
      point.y = 1024 * rand() / (RAND_MAX + 1.0f);
      point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;

    for (auto& point : *cloud_in)
      std::cout << point << std::endl;

    *cloud_out = *cloud_in;

    std::cout << "size:" << cloud_out->size() << std::endl;
    for (auto& point : *cloud_out)
      point.x += 0.7f   
    std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;

    for (auto& point : *cloud_out)
      std::cout << point << std::endl   
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final)    
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;




}