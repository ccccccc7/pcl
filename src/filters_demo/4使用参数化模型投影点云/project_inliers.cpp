#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

int
 main(int argc, char **argv)
 {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  reader.read("rgb_pt.pcd", *cloud);

  //定义模型系列对象，并填充对应的数据，使用ax+by+cz+d = 0的平面模型，其中a=b=d=0,c=1，也就是x-y平面
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  coefficients -> values.resize(4);
  coefficients -> values[0] = coefficients -> values[1] = 0;
  coefficients -> values[2] = 1.0;
  coefficients -> values[3] = 0;

  //创建ProjectInliers对象，并使用ModelCoefficients作为投影对象的模型参数
  pcl::ProjectInliers<pcl::PointXYZ> proj;  //创建投影滤波对象
  proj.setModelType(pcl::SACMODEL_PLANE);   //设置对象对应的投影模型
  proj.setInputCloud(cloud);                //设置输入点云
  proj.setModelCoefficients(coefficients);  //设置投影滤波存储结果
  proj.filter(*cloud_projected);            //投影滤波存储结果

  std::cerr << "Cloud after projection : " << std::endl;
  

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("rgb_projected.pcd", *cloud_projected, false);

  return (0); 
 }