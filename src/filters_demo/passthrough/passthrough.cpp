#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 定义读取对象
    pcl::PCDReader reader;
    // 读取点云文件
    reader.read<pcl::PointXYZ>("rgb_pt.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // 创建滤波器，对每个点分析的临*点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
    //个点的距离超出了*均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; //创建滤波器对象
    sor.setInputCloud(cloud);                          //设置待滤波的点云
    sor.setMeanK(50);                                  //设置在进行统计时考虑查询点临*点数
    sor.setStddevMulThresh(1.0);                       //设置判断是否为离群点的阀值
    sor.filter(*cloud_filtered);                       //存储

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("rgb_inliers.pcd", *cloud_filtered, false);

    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>("rgb_outliers.pcd", *cloud_filtered, false);

    return (0);
}