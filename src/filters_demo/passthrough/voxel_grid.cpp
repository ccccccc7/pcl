#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char **argv)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

    pcl::PCDReader reader;
    reader.read("rgb_pt.pcd", *cloud);

    std::cerr << "PointCloud before filtering" << cloud->width * cloud->height
     << " data points (" << pcl::getFieldsList (*cloud) << ").";

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;   //创建滤波对象
    sor.setInputCloud(cloud);                       //设置点云
    sor.setLeafSize(12.01f, 12.01f, 12.01f);           //设置滤波时创建体素大小为1立方体
    sor.filter(*cloud_filtered);                    //执行滤波处理，存储输出

    std::cerr << "PointCloud after filtering:" << cloud_filtered->width * cloud_filtered->height
        << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
    pcl::PCDWriter writer;
    writer.write("rgb_after.pcd", *cloud_filtered,
     Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

     return (0);
}