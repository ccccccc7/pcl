#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h> // 从一个点云中提取索引

int main(int argc, char **argv)
{

    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_p(new pcl::PointCloud<pcl::PointXYZ>), 
        cloud_f(new pcl::PointCloud<pcl::PointXYZ>),
        cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取pcd文件
    pcl::PCDReader reader;
    reader.read("../rgb_pt.pcd", *cloud_blob);

    //打印滤波前的点云个数
    std::cerr << "PointCloud Before filtering : " << cloud_blob->width * cloud_blob->height << "data points." << std::endl;

    // 下采样
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(10.0f, 10.0f, 10.0f);
    sor.filter(*cloud_filtered_blob);

    // 转换模板点云
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

    // 保存下采样的点云
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("../rgb_downsampled.pcd", *cloud_filtered, false);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg; //  创建分割对象
    seg.setOptimizeCoefficients(true);       //  设置对估计的模型参数进行优化处理
    seg.setModelType(pcl::SACMODEL_PLANE);   //  设置分割模型类型
    seg.setMaxIterations(1000);              //  设置最大迭代次数
    seg.setDistanceThreshold(0.01);          //  设置判断是否为模型内点的距离阀值

    //  设置extraction filter的实际参数
    pcl::ExtractIndices<pcl::PointXYZ> extract; //  创建点云提取对象

    int i = 0, nr_points = (int)cloud->points.size(); //点云总数

    while (cloud->points.size() > 0.3 * nr_points)
    {
        // 为了处理点云中包含的多个数据，在循环中执行该过程，在每次模型被提取后，保存剩余的点进行迭代
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        extract.setInputCloud(cloud); //  设置输入点云
        extract.setIndices(inliers);  //  设置分割后的内点为需要提取的点集
        extract.setNegative(false);   //  设置提取内点而非外点
        extract.filter(*cloud_p);     //  输出储存到cloud_p

        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        std::stringstream ss;
        ss << "../rgb_" << i << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud.swap(cloud_f);
        i++;
    }

    return (0);
}