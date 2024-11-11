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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::ReferenceFrame RFType;

bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.01f);
float descr_rad_ (0.8f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>);


pcl::PointCloud<PointType>::Ptr extractPointsInBoundingBox(const pcl::PointCloud<PointType>::Ptr& scene_cloud,
                                                           const pcl::PointCloud<PointType>::Ptr& bbox);



pcl::PointCloud<PointType>::Ptr createCube(float size, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0);
pcl::PointCloud<PointType>::Ptr createSphere(float radius, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0);
pcl::PointCloud<PointType>::Ptr createCylinder(float radius, float height, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0);

pcl::CorrespondencesPtr searchMatchingWithoutPointCloud(
    pcl::PointCloud<DescriptorType>::Ptr &model_descriptors,
    pcl::PointCloud<DescriptorType>::Ptr &scene_descriptors);

void computeKeypointsAndNormals(pcl::PointCloud<PointType>::Ptr& model,
                                pcl::PointCloud<PointType>::Ptr& keypoints,
                                pcl::PointCloud<NormalType>::Ptr& normals);

void computeSHOTDescriptors(pcl::PointCloud<PointType>::Ptr& model,
                            pcl::PointCloud<PointType>::Ptr& model_keypoints,
                            pcl::PointCloud<NormalType>::Ptr& model_normals,
                            pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors);


pcl::PointCloud<PointType>::Ptr extractBoundingBox(pcl::PointCloud<PointType>::Ptr cloud);

void funCallBack(const sensor_msgs::PointCloud2 &msg) {
  pcl::fromROSMsg(msg, *scene_cloud);
  ROS_INFO_STREAM ("size Point cloud: "<<scene_cloud->size());
}

visualization_msgs::Marker createBoundingBoxMarker(const pcl::PointCloud<PointType>::Ptr& bbox, int id);
int computeAverageColor(const pcl::PointCloud<PointType>::Ptr& cloud);


int main(int argc, char **argv) {
  ros::init(argc, argv, "results_work_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_segmentation", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1); // Publisher for markers
  ros::Subscriber sub = nh.subscribe("ros_cloud", 8, funCallBack);

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
    computeSHOTDescriptors(scene_cloud, scene_keypoints, scene_normals, scene_descriptors);

    // Создание модели куба (куб, сфера, цилиндр)
    //    pcl::PointCloud<PointType>::Ptr model = createCube(1.5, 0,255,0);
    //        pcl::PointCloud<PointType>::Ptr model = createSphere(1);
            pcl::PointCloud<PointType>::Ptr model = createCylinder(0.5, 1, 0, 255, 0);


    pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>);
    pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>);
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>);


    computeKeypointsAndNormals(model, model_keypoints, model_normals);
    computeSHOTDescriptors(model, model_keypoints, model_normals, model_descriptors);

    // Поиск соответствий между дескрипторами модели и сцены
    pcl::CorrespondencesPtr model_scene_corrs = searchMatchingWithoutPointCloud(model_descriptors, scene_descriptors);

    // Группировка по геометрическим признакам (Geometric Consistency Grouping)
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    if (use_hough_) {
      pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>);
      pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>);
      pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
      rf_est.setFindHoles(true);
      rf_est.setRadiusSearch(rf_rad_);
      rf_est.setInputCloud(model_keypoints);
      rf_est.setInputNormals(model_normals);
      rf_est.setSearchSurface(model);
      rf_est.compute(*model_rf);


      rf_est.setInputCloud(scene_keypoints);
      rf_est.setInputNormals(scene_normals);
      rf_est.setSearchSurface(scene_cloud);
      rf_est.compute(*scene_rf);

      pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
      clusterer.setHoughBinSize(cg_size_);
      clusterer.setHoughThreshold(cg_thresh_);
      clusterer.setUseInterpolation(true);
      clusterer.setUseDistanceWeight(false);
      clusterer.setInputCloud(model_keypoints);
      clusterer.setInputRf(model_rf);
      clusterer.setSceneCloud(scene_keypoints);
      clusterer.setSceneRf(scene_rf);
      clusterer.setModelSceneCorrespondences(model_scene_corrs);
      ROS_INFO_STREAM("model_scene_corrs: "<<model_scene_corrs->size());
      clusterer.recognize(rototranslations, clustered_corrs);
      ROS_INFO_STREAM("rototranslation: "<<rototranslations.size());
      ROS_INFO_STREAM("clusster_corrs: "<<clustered_corrs.size());
//      ros::Duration(5).sleep();

    } else {
      pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
      gc_clusterer.setGCSize(cg_size_);
      gc_clusterer.setGCThreshold(cg_thresh_);

      gc_clusterer.setInputCloud(model_keypoints);
      gc_clusterer.setSceneCloud(scene_keypoints);
      gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

      gc_clusterer.recognize(rototranslations, clustered_corrs);
    }

    // Определение bounding box для каждого обнаруженного объекта
    std::vector<pcl::PointCloud<PointType>::Ptr> boundingBoxes;
    for (const auto& rototranslation : rototranslations) {
      pcl::PointCloud<PointType>::Ptr transformed_model(new pcl::PointCloud<PointType>);

      pcl::transformPointCloud(*model, *transformed_model, rototranslation);

      pcl::PointCloud<PointType>::Ptr bbox = extractBoundingBox(transformed_model);

      //Алгоритм по оценке размера ограничивающей рамки. Необходим, чтобы фильтровать фигуры по размерам.
      PointType min_pt, max_pt;
      pcl::getMinMax3D(*bbox, min_pt, max_pt);
      double bbox_size = std::abs(max_pt.x - min_pt.x) * std::abs(max_pt.y - min_pt.y) * std::abs(max_pt.z - min_pt.z);
      // Определение размера ограничивающего параллелепипеда модели

      pcl::PointCloud<PointType>::Ptr model_bbox = extractBoundingBox(model);
      pcl::getMinMax3D(*model_bbox, min_pt, max_pt);
      double model_size = std::abs(max_pt.x - min_pt.x) * std::abs(max_pt.y - min_pt.y) * std::abs(max_pt.z - min_pt.z);


      // Сравнение размера с модельным размером
      if (std::abs(bbox_size - model_size) < 0.1) {

        pcl::PointCloud<PointType>::Ptr scene_bbox_cloud = extractPointsInBoundingBox(scene_cloud, bbox);

        if(computeAverageColor(scene_bbox_cloud) == computeAverageColor(model)){
          boundingBoxes.push_back(bbox);

        }
      }

    }


    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*model, output);
    output.header.frame_id = "map";
    pub.publish(output);

    // Publish the markers
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < boundingBoxes.size(); ++i) {
      visualization_msgs::Marker marker = createBoundingBoxMarker(boundingBoxes[i], i);
      marker_array.markers.push_back(marker);
    }
    marker_pub.publish(marker_array);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


pcl::CorrespondencesPtr searchMatchingWithoutPointCloud(
    pcl::PointCloud<DescriptorType>::Ptr &model_descriptors,
    pcl::PointCloud<DescriptorType>::Ptr &scene_descriptors) {

  pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud(model_descriptors);

  for (size_t i = 0; i < scene_descriptors->size(); ++i) {
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);

    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }

    int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);

    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.4f)
    {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
      ROS_INFO_STREAM("Found correspondence: Model index " << corr.index_match << ", Scene index " << corr.index_query << ", Distance " << corr.distance);
      model_scene_corrs->push_back(corr);
      ROS_INFO_STREAM("Total correspondences found: " << model_scene_corrs->size());
    }
  }

  return model_scene_corrs;
}


void computeKeypointsAndNormals(pcl::PointCloud<PointType>::Ptr& cloud,
                                pcl::PointCloud<PointType>::Ptr& keypoints,
                                pcl::PointCloud<NormalType>::Ptr& normals) {

  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud(cloud);
  uniform_sampling.setRadiusSearch(model_ss_);
  uniform_sampling.filter(*keypoints);

  ROS_INFO_STREAM(" Selected Keypoints: " << keypoints->size ());


  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch(10);
  norm_est.setInputCloud(cloud);
  norm_est.compute(*normals);
  ROS_INFO_STREAM(" Selected normals: " << normals->size ());

}




void computeSHOTDescriptors(pcl::PointCloud<PointType>::Ptr& cloud,
                            pcl::PointCloud<PointType>::Ptr& keypoints,
                            pcl::PointCloud<NormalType>::Ptr& normals,
                            pcl::PointCloud<DescriptorType>::Ptr& descriptors) {

  if (keypoints->empty() || normals->empty()) {
    ROS_ERROR("Model keypoints or normals are empty!");
    return;
  }

  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch(descr_rad_);
  descr_est.setInputCloud(keypoints);
  descr_est.setInputNormals(normals);
  descr_est.setSearchSurface(cloud);
  descr_est.compute(*descriptors);
}

pcl::PointCloud<PointType>::Ptr extractPointsInBoundingBox(const pcl::PointCloud<PointType>::Ptr& scene_cloud,
                                                           const pcl::PointCloud<PointType>::Ptr& bbox) {
  pcl::PointCloud<PointType>::Ptr scene_bbox_cloud(new pcl::PointCloud<PointType>);

  PointType min_pt, max_pt;
  pcl::getMinMax3D(*bbox, min_pt, max_pt);

  for (const auto& pt : scene_cloud->points) {
    if (pt.x >= min_pt.x && pt.x <= max_pt.x &&
        pt.y >= min_pt.y && pt.y <= max_pt.y &&
        pt.z >= min_pt.z && pt.z <= max_pt.z) {
      scene_bbox_cloud->points.push_back(pt);
    }
  }

  scene_bbox_cloud->width = scene_bbox_cloud->points.size();
  scene_bbox_cloud->height = 1;
  scene_bbox_cloud->is_dense = true;

  return scene_bbox_cloud;
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
visualization_msgs::Marker createBoundingBoxMarker(const pcl::PointCloud<PointType>::Ptr& bbox, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "bounding_boxes";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 2; // Толщина

  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;

  for (size_t i = 0; i < bbox->points.size(); ++i) {
    geometry_msgs::Point p;
    p.x = bbox->points[i].x;
    p.y = bbox->points[i].y;
    p.z = bbox->points[i].z;
    marker.points.push_back(p);
  }

  std::vector<std::pair<int, int>> edges = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7}
  };

  for (const auto& edge : edges) {
    marker.points.push_back(marker.points[edge.first]);
    marker.points.push_back(marker.points[edge.second]);
  }

  return marker;
}



int computeAverageColor(const pcl::PointCloud<PointType>::Ptr& cloud) {
  // Calculate average color of the bounding box
  int r = 0, g = 0, b = 0;
  for (const auto& pt : cloud->points) {
    r += pt.r;
    g += pt.g;
    b += pt.b;
  }
  int bbox_size = cloud->size();

  r /= bbox_size;
  g /= bbox_size;
  b /= bbox_size;


  return r+g+b;
}





pcl::PointCloud<PointType>::Ptr createCube(float size, uint8_t r, uint8_t g, uint8_t b) {
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  float half_size = size / 2.0;

  for (float x = -half_size; x <= half_size; x += 0.1) {
    for (float y = -half_size; y <= half_size; y += 0.1) {
      // Верхняя и нижняя грани
      for (float z : {-half_size, half_size}) {
        PointType point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.r = r;
        point.g = g;
        point.b = b;
        cloud->points.push_back(point);
      }
    }
  }

  for (float x = -half_size; x <= half_size; x += 0.1) {
    for (float z = -half_size; z <= half_size; z += 0.1) {
      // Передняя и задняя грани
      for (float y : {-half_size, half_size}) {
        PointType point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.r = r;
        point.g = g;
        point.b = b;
        cloud->points.push_back(point);
      }
    }
  }

  for (float y = -half_size; y <= half_size; y += 0.1) {
    for (float z = -half_size; z <= half_size; z += 0.1) {
      // Левая и правая грани
      for (float x : {-half_size, half_size}) {
        PointType point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.r = r;
        point.g = g;
        point.b = b;

        cloud->points.push_back(point);
      }
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}


pcl::PointCloud<PointType>::Ptr createSphere(float radius, uint8_t r, uint8_t g, uint8_t b) {
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  for (float phi = 0; phi <= M_PI; phi += M_PI / 18.0) {
    for (float theta = 0; theta < 2 * M_PI; theta += M_PI / 18.0) {
      PointType point;
      point.x = radius * sin(phi) * cos(theta);
      point.y = radius * sin(phi) * sin(theta);
      point.z = radius * cos(phi);
      point.r = r;
      point.g = g;
      point.b = b;
      cloud->points.push_back(point);
    }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

pcl::PointCloud<PointType>::Ptr createCylinder(float radius, float height, uint8_t r, uint8_t g, uint8_t b) {
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  for (float z = -height / 2; z <= height / 2; z += 0.2) {
    for (float angle = 0; angle <= 2 * M_PI; angle += M_PI / 10.0) {
      PointType point;
      point.x = radius * cos(angle);
      point.y = radius * sin(angle);
      point.z = z;
      point.r = r;
      point.g = g;
      point.b = b;
      cloud->points.push_back(point);
    }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}



