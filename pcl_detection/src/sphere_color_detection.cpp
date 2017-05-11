#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/console/parse.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <eigen3/Eigen/Dense>
#include <bullet/LinearMath/btQuaternion.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTRGB;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //publish the detected sphere
    sph_pub_ = n_.advertise<std_msgs::Float32MultiArray>("/sphere_coefs", 1);

    //publish the point cloud of the detected sphere
    sph_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/sphere_point_cloud", 1);

    //publish the marker for the detected sphere
    sph_marker_pub_ = n_.advertise<visualization_msgs::Marker>("/sphere_marker", 1);

    //subscribe to the point cloud from openni
    pcloud_sub_ = n_.subscribe ("/camera/depth_registered/points", 1, &SubscribeAndPublish::cloud_callback, this);
  }

  void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
     pcl::PCLPointCloud2 pcl_pc2;
     pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
     pcl::PointCloud<PointTRGB>::Ptr temp_cloud(new pcl::PointCloud<PointTRGB>);
     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
     // ROS_INFO("The width of point cloud is %d ", temp_cloud->width);
     // ROS_INFO("The height of point cloud is %d ", temp_cloud->height);
     // std::cerr << "PointCloud has: " << temp_cloud->points.size () << " data points." << std::endl;
     
     // All the objects needed
     pcl::PassThrough<PointTRGB> pass;
     pcl::NormalEstimation<PointTRGB, pcl::Normal> ne;
     pcl::SACSegmentationFromNormals<PointTRGB, pcl::Normal> seg; 
     pcl::PCDWriter writer;
     pcl::ExtractIndices<PointTRGB> extract;
     pcl::ExtractIndices<pcl::Normal> extract_normals;
     pcl::search::KdTree<PointTRGB>::Ptr tree (new pcl::search::KdTree<PointTRGB> ());
     
     // Datasets
     pcl::PointCloud<PointTRGB>::Ptr cloud_filtered_color (new pcl::PointCloud<PointTRGB>);
     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_color (new pcl::PointCloud<pcl::Normal>);
     pcl::PointCloud<PointTRGB>::Ptr cloud_filtered (new pcl::PointCloud<PointTRGB>);
     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
     pcl::PointCloud<PointTRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<PointTRGB>);
     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
     pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_sphere (new pcl::ModelCoefficients);
     pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_sphere (new pcl::PointIndices);

     // build the condition using the color range for green
     int rMax = 100;
     int rMin = 0;
     int gMax = 256;
     int gMin = 150;
     int bMax = 100;
     int bMin = 0;
     pcl::ConditionAnd<PointTRGB>::Ptr color_cond (new pcl::ConditionAnd<PointTRGB> ());
     color_cond->addComparison (pcl::PackedRGBComparison<PointTRGB>::Ptr (new pcl::PackedRGBComparison<PointTRGB> ("r", pcl::ComparisonOps::LT, rMax)));
     color_cond->addComparison (pcl::PackedRGBComparison<PointTRGB>::Ptr (new pcl::PackedRGBComparison<PointTRGB> ("r", pcl::ComparisonOps::GT, rMin)));
     color_cond->addComparison (pcl::PackedRGBComparison<PointTRGB>::Ptr (new pcl::PackedRGBComparison<PointTRGB> ("g", pcl::ComparisonOps::LT, gMax)));
     color_cond->addComparison (pcl::PackedRGBComparison<PointTRGB>::Ptr (new pcl::PackedRGBComparison<PointTRGB> ("g", pcl::ComparisonOps::GT, gMin)));
     color_cond->addComparison (pcl::PackedRGBComparison<PointTRGB>::Ptr (new pcl::PackedRGBComparison<PointTRGB> ("b", pcl::ComparisonOps::LT, bMax)));
     color_cond->addComparison (pcl::PackedRGBComparison<PointTRGB>::Ptr (new pcl::PackedRGBComparison<PointTRGB> ("b", pcl::ComparisonOps::GT, bMin)));

     // build the color filter
     pcl::ConditionalRemoval<PointTRGB> condrem;
     condrem.setCondition(color_cond);
     condrem.setInputCloud (temp_cloud);
     condrem.setKeepOrganized(false); 

     // apply filter
     condrem.filter (*cloud_filtered_color); 
     // ROS_INFO("The width of point cloud after color_filtering is %d ", cloud_filtered_color->width);
     // ROS_INFO("The height of point cloud after color_filtering is %d ", cloud_filtered_color->height);
     // std::cerr << "PointCloud after color_filtering has: " << cloud_filtered_color->points.size () << " data points." << std::endl;

     // Build a passthrough filter to remove spurious NaNs
     pass.setInputCloud (cloud_filtered_color);
     pass.setFilterFieldName ("z");
     pass.setFilterLimits (0, 1.5);
     pass.filter (*cloud_filtered);
     // std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;


     // Estimate point normals
     ne.setSearchMethod (tree);
     ne.setInputCloud (cloud_filtered);
     ne.setKSearch (50);
     ne.compute (*cloud_normals);

     /*
     /*
     // Create the segmentation object for the planar model and set all the parameters
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
     seg.setNormalDistanceWeight (0.1);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations (100);
     seg.setDistanceThreshold (0.03);
     seg.setInputCloud (cloud_filtered);
     seg.setInputNormals (cloud_normals);
     // Obtain the plane inliers and coefficients
     seg.segment (*inliers_plane, *coefficients_plane);
     // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

     // Extract the planar inliers from the input cloud
     extract.setInputCloud (cloud_filtered);
     extract.setIndices (inliers_plane);
     extract.setNegative (false);
 
     // Write the planar inliers to disk
     pcl::PointCloud<PointTRGB>::Ptr cloud_plane (new pcl::PointCloud<PointTRGB> ());
     extract.filter (*cloud_plane);
     // std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
     // writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

     // Remove the planar inliers, extract the rest
     extract.setNegative (true);
     extract.filter (*cloud_filtered2);
     extract_normals.setNegative (true);
     extract_normals.setInputCloud (cloud_normals);
     extract_normals.setIndices (inliers_plane);
     extract_normals.filter (*cloud_normals2); 
     */

     // Create the segmentation object for sphere segmentation and set all the parameters
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_SPHERE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setNormalDistanceWeight (0.1);
     seg.setMaxIterations (100000);
     seg.setDistanceThreshold (0.05);
     seg.setRadiusLimits (0.02, 0.05);
     seg.setInputCloud (cloud_filtered);
     seg.setInputNormals (cloud_normals); 

     // Obtain the sphere inliers and coefficients
     seg.segment (*inliers_sphere, *coefficients_sphere);
     std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl; 

     // Write the sphere inliers to disk
     extract.setInputCloud (cloud_filtered);
     extract.setIndices (inliers_sphere);
     extract.setNegative (false);
     pcl::PointCloud<PointTRGB>::Ptr cloud_sphere (new pcl::PointCloud<PointTRGB> ());
     extract.filter (*cloud_sphere);

     // publish pointcloud to ros
     sensor_msgs::PointCloud2 rosCloud;
     pcl::toROSMsg(*cloud_sphere, rosCloud);
     sph_cloud_pub_.publish(rosCloud);

     // pcl::getMinMax
     PointTRGB minPt, maxPt;
     pcl::getMinMax3D (*cloud_sphere, minPt, maxPt);
     float height = maxPt.y - minPt.y;
     // std::cout << "Max x: " << maxPt.x << std::endl;
     // std::cout << "Min x: " << minPt.x << std::endl;


     // publish a topic for the detected cloud_filtered
     std_msgs::Float32MultiArray msg_to_pub;

     // set up dimensions
     msg_to_pub.layout.dim.push_back(std_msgs::MultiArrayDimension());
     msg_to_pub.layout.dim[0].size = 4;
     msg_to_pub.layout.dim[0].stride = 1;
     msg_to_pub.layout.dim[0].label = "sphere_coefs";

     if (cloud_filtered->points.empty() )
         std::cerr << "Using color and distance filters no ball found!";
     else
     {
         float estimated_x = (maxPt.x + minPt.x) / 2.0;
         float estimated_y = (maxPt.y + minPt.y) / 2.0;
         float estimated_z = (maxPt.z + minPt.z) / 2.0;
         float estimated_radius = (maxPt.x - minPt.x) / 2.0;
         msg_to_pub.data.clear();
         msg_to_pub.data.push_back(estimated_x);
         msg_to_pub.data.push_back(estimated_y);
         msg_to_pub.data.push_back(estimated_z);
         msg_to_pub.data.push_back(estimated_radius);
         sph_pub_.publish(msg_to_pub);
     }



     if (cloud_sphere->points.empty ()) 
           std::cerr << "Can't find the spherical component." << std::endl;
     else
     {
           std::cerr << "PointCloud representing the spherical component: " << cloud_sphere->points.size () << " data points." << std::endl;
	  // writer.write ("table_scene_mug_stereo_textured_sphere.pcd", *cloud_sphere, false);
     }	
     
}

private:
  ros::NodeHandle n_; 
  ros::Publisher sph_pub_;
  ros::Publisher sph_cloud_pub_;
  ros::Publisher sph_marker_pub_;
  ros::Subscriber pcloud_sub_;

};//End of class SubscribeAndPublish



int main (int argc, char** argv) {
     ros::init (argc, argv, "cloud_sub");
     //Create an object of class to publish sphere coefficients
     SubscribeAndPublish SAPObject;
     ros::spin();
 }
