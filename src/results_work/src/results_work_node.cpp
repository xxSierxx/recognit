#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/features/shot_omp.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>


typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::ReferenceFrame RFType;

pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>);

pcl::PointCloud<PointType>::Ptr createCube(double size);
pcl::PointCloud<PointType>::Ptr createSphere(double radius);
pcl::PointCloud<PointType>::Ptr createCylinder(double radius, double height);
pcl::CorrespondencesPtr searchMatchingWithoutPointCloud(
    pcl::PointCloud<DescriptorType>::Ptr &model_descriptors,
    pcl::PointCloud<DescriptorType>::Ptr &scene_descriptors);

void computeKeypointsAndNormals(pcl::PointCloud<PointType>::Ptr& cloud,
                                pcl::PointCloud<PointType>::Ptr& keypoints,
                                pcl::PointCloud<NormalType>::Ptr& normals,
                                float uniform_sampling_size = 0.01);

void computeSHOTDescriptors(pcl::PointCloud<PointType>::Ptr& keypoints,
                            pcl::PointCloud<NormalType>::Ptr& normals,
                            pcl::PointCloud<PointType>::Ptr& model_keypoints,
                            pcl::PointCloud<NormalType>::Ptr& model_normals,
                            pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors);

pcl::PointCloud<PointType>::Ptr extractBoundingBox(pcl::PointCloud<PointType>::Ptr cloud);

void funCallBack(const sensor_msgs::PointCloud2 &msg) {
  pcl::fromROSMsg(msg, *scene_cloud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "results_work_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_segmentation", 1);
  ros::Subscriber sub = nh.subscribe("ros_cloud", 8, funCallBack);

  bool use_hough_ (true);
  float model_ss_ (0.01f);
  float scene_ss_ (0.03f);
  float descr_rad_ (0.02f);
  float rf_rad_ (0.015f);
  float cg_size_ (0.01f);
  float cg_thresh_ (5.0f);

  ros::Rate r(1);

  while (ros::ok()) {
    if (scene_cloud->empty()) {
      ros::spinOnce();
      r.sleep();
      continue;
    }

    // Вычисление ключевых точек и дескрипторов для сцены
    pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>);
    pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>);
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>);

    computeKeypointsAndNormals(scene_cloud, scene_keypoints, scene_normals);
    computeSHOTDescriptors(scene_cloud, scene_normals, scene_keypoints, scene_normals, scene_descriptors);

    // Определяем и создаем модель куба (куб, сфера, цилиндр)
    pcl::PointCloud<PointType>::Ptr model = createCube(0.1); // создаем модели куба
    pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>);
    pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>);
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>);

    computeKeypointsAndNormals(model, model_keypoints, model_normals);
    computeSHOTDescriptors(model, model_normals, model_keypoints, model_normals, model_descriptors);

    // Поиск соответствий между дескрипторами модели и сцены
    pcl::CorrespondencesPtr model_scene_corrs = searchMatchingWithoutPointCloud(model_descriptors, scene_descriptors);

    // Группировка по геометрическим признакам (Geometric Consistency Grouping)
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene_cloud);
    rf_est.compute (*scene_rf);


    if (use_hough_)
    {
      //
      //  Compute (Keypoints) Reference Frames only for Hough
      //
      pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
      pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

      pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
      rf_est.setFindHoles (true);
      rf_est.setRadiusSearch (rf_rad_);

      rf_est.setInputCloud (model_keypoints);
      rf_est.setInputNormals (model_normals);
      rf_est.setSearchSurface (model);
      rf_est.compute (*model_rf);

      rf_est.setInputCloud (scene_keypoints);
      rf_est.setInputNormals (scene_normals);
      rf_est.setSearchSurface (scene_cloud);
      rf_est.compute (*scene_rf);

      //  Clustering
      pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
      clusterer.setHoughBinSize (cg_size_);
      clusterer.setHoughThreshold (cg_thresh_);
      clusterer.setUseInterpolation (true);
      clusterer.setUseDistanceWeight (false);

      clusterer.setInputCloud (model_keypoints);
      clusterer.setInputRf (model_rf);
      clusterer.setSceneCloud (scene_keypoints);
      clusterer.setSceneRf (scene_rf);
      clusterer.setModelSceneCorrespondences (model_scene_corrs);

      //clusterer.cluster (clustered_corrs);
      clusterer.recognize (rototranslations, clustered_corrs);
    }
//    else
//    {
//      pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
//      gc_clusterer.setGCSize (cg_size_);
//      gc_clusterer.setGCThreshold (cg_thresh_);

//      gc_clusterer.setInputCloud (model_keypoints);
//      gc_clusterer.setSceneCloud (scene_keypoints);
//      gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

//      gc_clusterer.recognize (rototranslations, clustered_corrs);
//    }

    // Определение bounding box для каждого обнаруженного объекта
    std::vector<pcl::PointCloud<PointType>::Ptr> boundingBoxes;

    for (const auto& rototranslation : rototranslations) {
      pcl::PointCloud<PointType>::Ptr transformed_model(new pcl::PointCloud<PointType>);
      pcl::transformPointCloud(*model, *transformed_model, rototranslation);
      pcl::PointCloud<PointType>::Ptr bbox = extractBoundingBox(transformed_model);
      boundingBoxes.push_back(bbox);
    }

    // Publish the segmented point cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*scene_cloud, output);
    output.header.frame_id = "camera_link";
    pub.publish(output);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

pcl::PointCloud<PointType>::Ptr createCube(double size) {
  pcl::PointCloud<PointType>::Ptr cube(new pcl::PointCloud<PointType>());

  // Вершины куба
  PointType pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8;
  pt1.x = -size / 2; pt1.y = -size / 2; pt1.z = -size / 2;
  pt2.x = size / 2; pt2.y = -size / 2; pt2.z = -size / 2;
  pt3.x = size / 2; pt3.y = size / 2; pt3.z = -size / 2;
  pt4.x = -size / 2; pt4.y = size / 2; pt4.z = -size / 2;
  pt5.x = -size / 2; pt5.y = -size / 2; pt5.z = size / 2;
  pt6.x = size / 2; pt6.y = -size / 2; pt6.z = size / 2;
  pt7.x = size / 2; pt7.y = size / 2; pt7.z = size / 2;
  pt8.x = -size / 2; pt8.y = size / 2; pt8.z = size / 2;

  cube->points.push_back(pt1);
  cube->points.push_back(pt2);
  cube->points.push_back(pt3);
  cube->points.push_back(pt4);
  cube->points.push_back(pt5);
  cube->points.push_back(pt6);
  cube->points.push_back(pt7);
  cube->points.push_back(pt8);

  return cube;
}

pcl::PointCloud<PointType>::Ptr createSphere(double radius) {
  pcl::PointCloud<PointType>::Ptr sphere(new pcl::PointCloud<PointType>());

  int resolution = 20; // Разрешение сферы

  for (int i = 0; i < resolution; ++i) {
    for (int j = 0; j < resolution; ++j) {
      double theta = 2 * M_PI * i / resolution;
      double phi = M_PI * j / resolution;
      PointType pt;
      pt.x = radius * sin(phi) * cos(theta);
      pt.y = radius * sin(phi) * sin(theta);
      pt.z = radius * cos(phi);
      sphere->points.push_back(pt);
    }
  }

  return sphere;
}

pcl::PointCloud<PointType>::Ptr createCylinder(double radius, double height) {
  pcl::PointCloud<PointType>::Ptr cylinder(new pcl::PointCloud<PointType>());

  int resolution = 20; // Разрешение цилиндра

  for (int i = 0; i < resolution; ++i) {
    double theta = 2 * M_PI * i / resolution;
    for (double h = -height / 2; h <= height / 2; h += height / resolution) {
      PointType pt;
      pt.x = radius * cos(theta);
      pt.y = radius * sin(theta);
      pt.z = h;
      cylinder->points.push_back(pt);
    }
  }

  return cylinder;
}

pcl::CorrespondencesPtr searchMatchingWithoutPointCloud(
    pcl::PointCloud<DescriptorType>::Ptr &model_descriptors,
    pcl::PointCloud<DescriptorType>::Ptr &scene_descriptors) {
  pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

  //    pcl::KdTree<PointType> test;
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud(scene_descriptors);

  for (size_t i = 0; i < model_descriptors->size(); ++i) {
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);
    int found_neighs = match_search.nearestKSearch(model_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) // 0.25 is a threshold, can be tuned
    {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back(corr);
    }
  }

  return model_scene_corrs;
}

void computeKeypointsAndNormals(pcl::PointCloud<PointType>::Ptr& cloud,
                                pcl::PointCloud<PointType>::Ptr& keypoints,
                                pcl::PointCloud<NormalType>::Ptr& normals,
                                float uniform_sampling_size) {
  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud(cloud);
  uniform_sampling.setRadiusSearch(uniform_sampling_size);


  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch(10);
  norm_est.setInputCloud(keypoints);
  norm_est.compute(*normals);
}

void computeSHOTDescriptors(pcl::PointCloud<PointType>::Ptr& keypoints,
                            pcl::PointCloud<NormalType>::Ptr& normals,
                            pcl::PointCloud<PointType>::Ptr& model_keypoints,
                            pcl::PointCloud<NormalType>::Ptr& model_normals,
                            pcl::PointCloud<DescriptorType>::Ptr& descriptors) {
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch(0.02);
  descr_est.setInputCloud(model_keypoints);
  descr_est.setInputNormals(model_normals);
  descr_est.setSearchSurface(keypoints);
  descr_est.compute(*descriptors);
}

void ros_fun(const pcl::PointCloud<PointType>::ConstPtr& cloud, sensor_msgs::PointCloud2& pointCloud) {
  pcl::toROSMsg(*cloud, pointCloud);
  pointCloud.header.frame_id = "camera_link";
}


pcl::PointCloud<PointType>::Ptr extractBoundingBox(pcl::PointCloud<PointType>::Ptr cloud) {
  pcl::PointCloud<PointType>::Ptr boundingBox(new pcl::PointCloud<PointType>);
  PointType min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);

  PointType pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8;
  pt1.x = min_pt.x; pt1.y = min_pt.y; pt1.z = min_pt.z;
  pt2.x = max_pt.x; pt2.y = min_pt.y; pt2.z = min_pt.z;
  pt3.x = max_pt.x; pt3.y = max_pt.y; pt3.z = min_pt.z;
  pt4.x = min_pt.x; pt4.y = max_pt.y; pt4.z = min_pt.z;
  pt5.x = min_pt.x; pt5.y = min_pt.y; pt5.z = max_pt.z;
  pt6.x = max_pt.x; pt6.y = min_pt.y; pt6.z = max_pt.z;
  pt7.x = max_pt.x; pt7.y = max_pt.y; pt7.z = max_pt.z;
  pt8.x = min_pt.x; pt8.y = max_pt.y; pt8.z = max_pt.z;

  boundingBox->points.push_back(pt1);
  boundingBox->points.push_back(pt2);
  boundingBox->points.push_back(pt3);
  boundingBox->points.push_back(pt4);
  boundingBox->points.push_back(pt5);
  boundingBox->points.push_back(pt6);
  boundingBox->points.push_back(pt7);
  boundingBox->points.push_back(pt8);

  return boundingBox;
}


