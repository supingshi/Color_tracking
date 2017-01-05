#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_types_conversion.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/impl/point_types.hpp>
#include <math.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include "std_msgs/String.h"
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <ros/package.h>
#include <custom_msgs/Object.h>
#include <std_msgs/Int8.h>
#include <custom_msgs/color_center.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


static const std::string OPENCV_WINDOW = "Image window";

ros::Publisher pub_object, cloud_publisher;
ros::ServiceServer service;
bool base_found (false);

cv::Mat result;
cv::Mat img_hsv;
cv::Mat img_binary;

int cloud_out_seq = 0;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorfilterptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

sensor_msgs::PointCloud2ConstPtr latestMsg (new sensor_msgs::PointCloud2);

std::string path = ros::package::getPath("vision");

bool hasGottenMsg = false;

void transformToBaseLink(const pcl::PointCloud<pcl::PointXYZRGBA> & input, pcl::PointCloud<pcl::PointXYZRGBA> & pcl_out){
  const tf::TransformListener listener;

  //Retrive correct time stamp from cloud
  ros::Time timePCL;
  pcl_conversions::fromPCL(input.header.stamp, timePCL);
  pcl_out.header.frame_id = "/base_link";
 
  try
  {
    listener.waitForTransform("/base_link", "/camera_link", timePCL, ros::Duration(1.0));
    pcl_ros::transformPointCloud("/base_link", input, pcl_out, listener);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("Received an exception trying to transform a point from \"camera_link\" to \"base_link\": %s", ex.what());
  }

}



bool center_pos(int colorID, geometry_msgs::PointStamped & obj_base)
{
  ros::spinOnce();

  colorfilterptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
  cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
  transformedCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
  filteredCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  // Convert to PCL data type
  pcl_conversions::toPCL(*latestMsg, *cloud);
  pcl::fromPCLPointCloud2(*cloud, *cloud_xyzrgb);

  //should be done by pcl::fromROSMsg (*input, *cloudPcl);

  int size = cloud_xyzrgb->size();


  if (size < 200) {
    return false;
  }


  if (cloud_xyzrgb->isOrganized()) {
      result = cv::Mat(cloud_xyzrgb->height, cloud_xyzrgb->width, CV_8UC4);

      if (!cloud_xyzrgb->empty()) {

          for (int h=0; h<result.rows; h++) {
              for (int w=0; w<result.cols; w++) {
               
                  pcl::PointXYZRGBA point = cloud_xyzrgb->at(w,h);

                  result.at<cv::Vec4b>(h,w)[0] = point.b;
                  result.at<cv::Vec4b>(h,w)[1] = point.g;
                  result.at<cv::Vec4b>(h,w)[2] = point.r;
                  result.at<cv::Vec4b>(h,w)[3] = point.a;
              }
          }
      }
  }

  cv::cvtColor(result,img_hsv,CV_BGR2HSV);
  result.release();


  cv::Mat binary_img_array;
  switch (colorID) {
    case 0: cv::inRange(img_hsv,cv::Scalar(145,0,0.2*255), cv::Scalar(179,0.45*255,0.7*255),binary_img_array); break;
    case 1: cv::inRange(img_hsv,cv::Scalar(0,0.55*255,0.3*255), cv::Scalar(10,255,0.7*255),binary_img_array); break;
    case 2: cv::inRange(img_hsv,cv::Scalar(40,0.4*255,0.12*255), cv::Scalar(70,0.95*255,0.82*255),binary_img_array); break;
    case 3: cv::inRange(img_hsv,cv::Scalar(70,0.33*255,0.1*255),  cv::Scalar(110,0.83*255,0.5*255),binary_img_array); break;
    case 4: cv::inRange(img_hsv,cv::Scalar(16,0.5*255,0.53*255),cv::Scalar(30,255,0.93*255),binary_img_array); break;
    case 5: cv::inRange(img_hsv,cv::Scalar(10,0.62*255,0.55*255),cv::Scalar(16,255,0.95*255),binary_img_array); break;

    default: ROS_ERROR("Color NOT found.");
  }

  /*
  colorfilterptr->height= cloud_xyzrgb->height;
  colorfilterptr->width=cloud_xyzrgb->width;
  colorfilterptr->points.resize(cloud_xyzrgb->size());
  colorfilterptr->header.frame_id = cloud_xyzrgb->header.frame_id;
  */

  int cell_count = 0;


  for (int h=0; h<binary_img_array.rows; h++) {     
    for (int w=0; w<binary_img_array.cols; w++) {
      if(binary_img_array.at<unsigned char>(h,w) > 0)
      {
        if(std::isfinite(cloud_xyzrgb->at(w,h).x))
        {
          colorfilterptr->points.push_back(cloud_xyzrgb->at(w,h));
         // colorfilterptr->at(w,h) = cloud_xyzrgb->at(w,h);
        }
        cell_count++;
      }
    }
  }


  /*
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_down_sampled (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (colorfilterptr);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloud_down_sampled);

  //std::cerr << "downsampleed cloud size = " << cloud_down_sampled->points.size() << std::endl;
  std::cerr << "hello3\n";
  */

  colorfilterptr->header.frame_id = "/camera_link";
  colorfilterptr->header.stamp = pcl_conversions::toPCL(ros::Time());

  transformToBaseLink(*colorfilterptr, *transformedCloud);

  std::cerr << "transformedCloud->points.size()=" << transformedCloud->points.size() << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGBA> pass;

  pass.setInputCloud (transformedCloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.01, 1.0);
  pass.filter (*filteredCloud);

  /*
  sensor_msgs::PointCloud2::Ptr cloud_out(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*filteredCloud, *cloud_out);
  cloud_out->header.seq = cloud_out_seq++;
  cloud_out->header.frame_id = "/base_link";
  cloud_out->header.stamp = ros::Time();
  cloud_publisher.publish(*cloud_out);
  */


  std::cerr << "filteredCloud->points.size()=" << filteredCloud->points.size() << std::endl;

  /*
  std::stringstream ss;
  ss << "scene" << colorID << ".pcd";
  std::string str = ss.str();

  std::string path1 = path + "/pcd_files/scene/" + str;
  pcl::io::savePCDFile(path1, *filteredCloud, false);
  */

  if (filteredCloud->points.size() > 1000 )
  {
    std::cerr << "Colour " << colorID << std::endl;
    cell_count = 0;


    tf::TransformListener listener;
    tf::StampedTransform transform;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid  (*filteredCloud, centroid);
    

    if (isnan(centroid[0]))
    {
      return false;
    }

    obj_base.header.stamp = ros::Time();
    obj_base.header.frame_id = "/base_link";
    obj_base.point.x = centroid[0];
    obj_base.point.y = centroid[1];
    obj_base.point.z = centroid[2];

    return true;
  }
  return false;
}




bool get_color_center(custom_msgs::color_center::Request  &req, custom_msgs::color_center::Response &res)
{
  //req.i = COLOR ID
  hasGottenMsg = false;
  ros::spinOnce();
  
  int i = 0;
  while (!hasGottenMsg && i < 10) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    std::cerr << "sleeping\n";
    i++;
  }

  geometry_msgs::PointStamped obj_base;
  std::cerr << "finding position :" << std::endl;
  bool obj_found = center_pos(req.i, obj_base);
  if (obj_found)
  {
    res.x = obj_base.point.x;
    res.y = obj_base.point.y;
    res.z = obj_base.point.z;

    sensor_msgs::PointCloud2 coloredCloud;
    pcl::toROSMsg(*filteredCloud, coloredCloud);


    res.coloredCloud = coloredCloud;
    std::cerr << "FOUND" << std::endl;

    return true;
  }
    std::cerr << "NOT FOUND" << std::endl;

  return false;
 
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  latestMsg = sensor_msgs::PointCloud2ConstPtr (new sensor_msgs::PointCloud2);
  latestMsg = cloud_msg;
  hasGottenMsg = true;
}



int main (int argc, char** argv)
{
  ros::init (argc, argv, "color_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  //cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);

  service = nh.advertiseService("get_color_center", get_color_center);

  ros::spin();

  return 0;
}
