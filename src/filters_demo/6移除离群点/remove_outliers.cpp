//
// Created by 东山 on 2022/4/2.
//

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>

int
main(int argc, char **argv) {

    if (argc != 2) {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    }

    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read("../rgb_pt.pcd", *cloud_blob);

    // 转换模板点云
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

    if (strcmp(argv[1], "-r") == 0) {
        //半径1内必须有5个邻居
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlierRemoval;    //创建滤波器
        outlierRemoval.setInputCloud(cloud);                  //设置输入点云
        outlierRemoval.setRadiusSearch(1);                    //设置在1半径范围内找邻近点
        outlierRemoval.setMinNeighborsInRadius(5);           //设置查询点等的邻近点集数小于2的删除
        outlierRemoval.filter(*cloud_filtered);
    } else if (strcmp(argv[1], "-c") == 0) {
        //创建条件定义对象
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());

        //为条件定义对象添加比较算子,z字段大于0小于0.8
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.0)));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));

        pcl::ConditionalRemoval<pcl::PointXYZ> conditionalRemoval;
        conditionalRemoval.setCondition(range_cond);
        conditionalRemoval.setInputCloud(cloud);
        conditionalRemoval.setKeepOrganized(true);
        conditionalRemoval.filter(*cloud_filtered);

    } else {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }

    std::cerr << "Cloud before filtering: " << cloud->width * cloud->height << "data points." << std::endl;
    // display pointcloud after filtering
    std::cerr << "Cloud after filtering: " << cloud_filtered->height * cloud_filtered->width << "data points." << std::endl;
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("../rgb_filtered.pcd", *cloud_filtered, false);
    return (0);
}

