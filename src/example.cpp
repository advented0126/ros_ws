#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>

#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Publisher pub;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
 /* pcl::PCLPointCloud2 cloud_filtered;

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Publish the data
  pub.publish (cloud_filtered);*/
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();
    sensor_msgs::PointCloud2 input_temp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fore_ground(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr after(new pcl::PointCloud<pcl::PointXYZRGB>);
     input_temp = *cloud;
     pcl::fromROSMsg(input_temp,*temp_cloud);



    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/ted/catkin_ws/src/my_pcl_tutorial/src/1701494893543760.pcd", *fore_ground);
    
    float resolution =0.1f;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(resolution);
   octree.setInputCloud(temp_cloud);//输入点云cloud1
   octree.addPointsFromInputCloud();//点云cloud1构建八叉树
    octree.switchBuffers();//交换八叉树缓存，但是cloud1对应的八叉树结构仍在内存中
    octree.setInputCloud(fore_ground);//输入点云cloud2
   octree.addPointsFromInputCloud();
    
    std::vector<int> newPointIdxVector;//存储新加点的索引
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);//获取新增点的索引

    //将新增点按照索引从cloud2中取出，放到cloud12_bianhua中
   for (size_t i = 0; i < newPointIdxVector.size(); ++i)
   {
        after->push_back(temp_cloud->points[newPointIdxVector[i]]);
    }
       sensor_msgs::PointCloud2 output_temp;
       pcl::PointCloud<pcl::PointXYZRGB> adjust_pcl;
       adjust_pcl =*after;
       pcl::toROSMsg(adjust_pcl,output_temp);
      output_temp.header=cloud->header;
      end_ = ros::WallTime::now();
      double execution_time = (end_ - start_).toNSec() * 1e-6;
      ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
       pub.publish(output_temp);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  ros::Rate rate(10);
  // Spin
  ros::spin ();
}