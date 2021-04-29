#include "evaluator.h"

namespace roi_viewpoint_planner
{

Evaluator::Evaluator(std::shared_ptr<PlannerInterface> planner, ros::NodeHandle &nh, ros::NodeHandle &nhp, bool gt_comparison, bool use_pcl)
  : planner(planner), gt_comparison(gt_comparison), use_pcl(use_pcl),
    viewer(nullptr), vp1(0), vp2(1), viewer_initialized(false), server(nhp),
    visualizeThread(&Evaluator::visualizeLoop, this)
{
  gt_pub = nhp.advertise<visualization_msgs::Marker>("roi_gt", 10, true);
  gt_fruit_pub = nhp.advertise<octomap_msgs::Octomap>("gt_fruits", 10, true);
  gt_fruits_inflated_pub = nhp.advertise<octomap_msgs::Octomap>("gt_fruits_inflated", 10, true);
  if (gt_comparison)
  {
    if (!nh.param<std::string>("/world_name", world_name, ""))
    {
      ROS_WARN("World name not specified, cannot load ground truth; Evaluator state invalid");
      return;
    }
    else if (!readGroundtruth())
    {
      ROS_WARN("Groundtruth could not be read; Evaluator state invalid");
      return;
    }

    // Register publishers and subscribers

    publishCubeVisualization(gt_pub, roi_locations, roi_sizes, COLOR_GREEN, "gt_rois");
  }

  if (use_pcl)
  {
    while(!viewer_initialized.load())
    {
      ROS_INFO("Waiting for viewer initialization...");
      ros::Duration(0.1).sleep();
    }

    updateVisualizerGroundtruth();
  }

  server.setCallback(boost::bind(&Evaluator::reconfigureCallback, this, boost::placeholders::_1, boost::placeholders::_2));
}

bool Evaluator::readGroundtruth()
{
  if (!gt_comparison)
    return false;

  gtLoader.reset(new GtOctreeLoader(world_name, planner->getTreeResolution()));

  octomap_msgs::Octomap fruit_ot_msg;
  fruit_ot_msg.header.frame_id = "world";
  fruit_ot_msg.header.stamp = ros::Time::now();
  std::shared_ptr<const octomap_vpp::CountingOcTree> gt_fruits = gtLoader->getIndexedFruitTree();
  bool msg_generated = octomap_msgs::fullMapToMsg(*gt_fruits, fruit_ot_msg);
  if (msg_generated)
  {
    gt_fruit_pub.publish(fruit_ot_msg);
  }
  std::shared_ptr<octomap_vpp::NearestRegionOcTree> gt_fruits_inflated = octomap_vpp::NearestRegionOcTree::createFromCountringOctree(*gt_fruits, 0.2);
  msg_generated = octomap_msgs::fullMapToMsg(*gt_fruits_inflated, fruit_ot_msg);
  if (msg_generated)
  {
    gt_fruits_inflated_pub.publish(fruit_ot_msg);
  }

  auto isGtOcc = [](const octomap_vpp::CountingOcTree &tree, const octomap_vpp::CountingOcTreeNode *node) { return true; };
  gt_pcl = gtLoader->getPclCloud();
  gt_clusters = gtLoader->getPclClusters();
  computeHullsAndVolumes(gt_pcl, *gt_clusters, gt_hulls_clouds, gt_hulls_polygons, gt_params.fruit_volumes, gt_centroids);

  gt_params.num_fruits = gtLoader->getNumFruits();
  gt_params.fruit_cell_counts = gtLoader->getFruitCellsCounts();

  return true;
}

/*void Evaluator::computeGroundtruthPCL()
{
  if (!gt_comparison || !use_pcl)
    return;

  gt_clusters.clear();
  gt_hulls_clouds.clear();
  gt_hulls_polygons.clear();
  gt_cluster_volumes.clear();

  clusterWithPCL(gt_pcl, gt_clusters);
  computeHullsAndVolumes(gt_pcl, gt_clusters, gt_hulls_clouds, gt_hulls_polygons, gt_cluster_volumes);
}*/

/*void Evaluator::computeDetectionsPCL()
{
  if (!use_pcl)
    return;

  clusters.clear();
  hulls_clouds.clear();
  hulls_polygons.clear();
  cluster_volumes.clear();

  clusterWithPCL(roi_pcl, clusters);
  computeHullsAndVolumes(roi_pcl, clusters, hulls_clouds, hulls_polygons, cluster_volumes);
}*/

void Evaluator::updateVisualizerGroundtruth()
{
  if (!gt_comparison || !use_pcl)
    return;

  gt_color_handler.reset(new pcl::visualization::PointCloudColorHandlerClusters<pcl::PointXYZ>(gt_pcl, *gt_clusters));
  viewer_mtx.lock();
  viewer->removeAllPointClouds(vp1);
  viewer->removeAllShapes(vp1);
  viewer->addPointCloud<pcl::PointXYZ> (gt_pcl, *gt_color_handler, "gt_cloud", vp1);
  if (config.show_hulls)
  {
    for (size_t i = 0; i < gt_clusters->size(); i++)
    {
      std::string name = "gt_cluster_" + std::to_string(i);
      viewer->addPolygonMesh<pcl::PointXYZ>(gt_hulls_clouds[i], gt_hulls_polygons[i], name, vp1);
    }
  }
  viewer_mtx.unlock();
}

void Evaluator::updateVisualizerDetections()
{
  if (viewer_initialized.load())
  {
    roi_color_handler.reset(new pcl::visualization::PointCloudColorHandlerClusters<pcl::PointXYZ>(roi_pcl, clusters));
    viewer_mtx.lock();
    viewer->removeAllPointClouds(vp2);
    viewer->removeAllShapes(vp2);
    viewer->addPointCloud<pcl::PointXYZ> (roi_pcl, *roi_color_handler, "roi_cloud", vp2);
    //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (roi_pcl, normal_cloud, 1, 0.02f, "roi_normals");
    if (config.show_hulls)
    {
      for (size_t i = 0; i < clusters.size(); i++)
      {
        std::string name = "cluster_" + std::to_string(i);
        viewer->addPolygonMesh<pcl::PointXYZ>(hulls_clouds[i], hulls_polygons[i], name, vp2);
      }
    }
    viewer_mtx.unlock();
  }
}

void Evaluator::reconfigureCallback(roi_viewpoint_planner::EvaluatorConfig &new_config, uint32_t level)
{
  if (new_config.min_cluster_size > new_config.max_cluster_size) // min cluster size must be smaller than max cluster size
  {
    if (level & (1 << 1)) // min_cluster_size was changed
      new_config.max_cluster_size = new_config.min_cluster_size;
    else //if (level & (1 << 2)) / max_cluster_size was changed
      new_config.min_cluster_size = new_config.max_cluster_size;
  }

  if (!new_config.curvature_test_flag && !new_config.residual_test_flag) // either curvature or residual test must be activated
  {
    if (level & (1 << 9)) // residual_test_flag was changed
      new_config.curvature_test_flag = true;
    else if (level & (1 << 8)) // curvature_test_flag was changed
      new_config.residual_test_flag = true;
  }

  config = new_config;

  if (use_pcl)
  {
    //computeGroundtruthPCL();
    //updateVisualizerGroundtruth();
    processDetectedRois();
    updateVisualizerDetections();
  }
}

void Evaluator::visualizeLoop()
{
  if (!use_pcl)
    return;

  viewer_mtx.lock();
  viewer = new pcl::visualization::PCLVisualizer("ROI viewer");
  if (gt_comparison)
  {
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, vp1);
    viewer->setBackgroundColor (0, 0, 0, vp1);
    viewer->addText ("Groundtruth", 10, 10, "vp1cap", vp1);
  }
  //viewer->createViewPortCamera(vp1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp2);
  viewer->setBackgroundColor (0.1, 0.1, 0.1, vp2);
  viewer->addText ("Detection", 10, 10, "vp2cap", vp2);
  //viewer->createViewPortCamera(vp2);
  viewer_initialized.store(true);
  viewer_mtx.unlock();
  for (ros::Rate rate(30); ros::ok(); rate.sleep())
  {
    viewer_mtx.lock();
    viewer->spinOnce();
    viewer_mtx.unlock();
  }
}

bool Evaluator::clusterWithPCL(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, std::vector<pcl::PointIndices> &clusters, pcl::PointCloud<pcl::Normal>::Ptr normal_cloud)
{
  if (input_cloud->empty())
  {
    ROS_WARN("Input cloud was emtpy; no clustering performed");
    return false;
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_cloud);

  if (config.clustering == roi_viewpoint_planner::Evaluator_EUCLIDEAN_CLUSTERING)
  {
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (config.cluster_tolerance);
    ec.setMinClusterSize (config.min_cluster_size);
    ec.setMaxClusterSize (config.max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_cloud);
    ec.extract (clusters);
  }
  else if (config.clustering == roi_viewpoint_planner::Evaluator_REGION_GROWING)
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(config.normal_search_radius);
    normal_cloud.reset(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normal_cloud);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
    rg.setInputCloud(input_cloud);
    rg.setSearchMethod(tree);
    rg.setInputNormals(normal_cloud);
    rg.setMinClusterSize(config.min_cluster_size);
    rg.setMaxClusterSize(config.max_cluster_size);
    rg.setSmoothnessThreshold(config.smoothness_threshold / 180.0f * M_PI);
    rg.setCurvatureThreshold(config.curvature_threshold);
    rg.setResidualThreshold(config.residual_threshold);
    rg.setSmoothModeFlag(config.smooth_mode_flag);
    rg.setCurvatureTestFlag(config.curvature_test_flag);
    rg.setResidualTestFlag(config.residual_test_flag);
    rg.extract(clusters);
  }
  else
  {
    ROS_ERROR("No valid clustering method was selected");
    return false;
  }
  return true;
}

bool Evaluator::computeHullsAndVolumes(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, const std::vector<pcl::PointIndices> &clusters,
                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &hulls_clouds,
                           std::vector<std::vector<pcl::Vertices>> &hulls_polygons,
                           std::vector<double> &cluster_volumes,
                           std::vector<pcl::PointXYZ> &centroids)
{
  if (input_cloud->empty() || clusters.empty())
    return false;

  for (const pcl::PointIndices &cluster : clusters)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> hull_polygons;

    pcl::PointIndicesPtr cluster_ptr(new pcl::PointIndices(cluster));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extr_inds;
    extr_inds.setInputCloud(input_cloud);
    extr_inds.setIndices(cluster_ptr);
    extr_inds.filter(*cluster_cloud);

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cluster_cloud);
    //hull.setIndices(cluster_ptr); // doesn't work -> use extract indices
    hull.setComputeAreaVolume(true);

    hull.reconstruct(*hull_cloud, hull_polygons);
    int dim = hull.getDimension();
    double vol;
    if (dim == 2)
    {
      const double thickness = 0.01; // Assume 1 cm thickness (might be reasonable to set to octomap resolution)
      vol = hull.getTotalArea() * thickness * 1000000; // to cm3
    }
    else // if dim == 3
    {
      vol = hull.getTotalVolume() * 1000000; // to cm3
    }

    ROS_INFO_STREAM("Hull dimension: " << dim << "Cluster volume: " << vol);

    hulls_clouds.push_back(hull_cloud);
    hulls_polygons.push_back(hull_polygons);
    cluster_volumes.push_back(vol);
    pcl::PointXYZ centroid;
    pcl::computeCentroid(*input_cloud, cluster.indices, centroid);
    centroids.push_back(centroid);
  }
  return true;
}

void Evaluator::computePairsAndDistances()
{
  distances.clear();
  roiPairs.clear();

  // Build up matrix with all pairs
  distances = boost::numeric::ublas::matrix<double>(roi_locations.size(), detected_locs.size());
  std::vector<IndexPair> indices;
  indices.reserve(roi_locations.size() * detected_locs.size());
  for (size_t i = 0; i < roi_locations.size(); i++)
  {
    for (size_t j = 0; j < detected_locs.size(); j++)
    {
      distances(i, j) = (roi_locations[i] - detected_locs[j]).norm();
      indices.push_back(IndexPair(i, j));
    }
  }

  auto indicesComp = [this](const IndexPair &a, const IndexPair &b)
  {
    return distances(a.gt_ind, a.det_ind) > distances(b.gt_ind, b.det_ind);
  };

  boost::dynamic_bitset<> usedGtPoints(roi_locations.size());
  boost::dynamic_bitset<> usedDetPoints(detected_locs.size());

  roiPairs.reserve(std::min(roi_locations.size(), detected_locs.size()));

  double MAX_DISTANCE = 0.2; // Maximal distance to be considered as same ROI

  for (std::make_heap(indices.begin(), indices.end(), indicesComp); !usedGtPoints.all() && !usedDetPoints.all(); std::pop_heap(indices.begin(), indices.end(), indicesComp), indices.pop_back())
  {
    const IndexPair &pair = indices.front();

    if (distances(pair.gt_ind, pair.det_ind) > MAX_DISTANCE)
      break;

    if (usedGtPoints.test(pair.gt_ind) || usedDetPoints.test(pair.det_ind))
      continue;

    roiPairs.push_back(pair);
    usedGtPoints.set(pair.gt_ind);
    usedDetPoints.set(pair.det_ind);
  }
}

EvaluationParameters Evaluator::processDetectedRois(bool save_pointcloud, size_t trial_num, size_t step)
{
  EvaluationParameters results;
  std::shared_ptr<octomap_vpp::RoiOcTree> roiTree = planner->getPlanningTree();
  if (!roiTree)
    return results;

  planner->getTreeMutex().lock();
  roiTree->computeRoiKeys();
  octomap::KeySet roi_keys = roiTree->getRoiKeys();
  planner->getTreeMutex().unlock();

  /*size_t detected_roi_clusters;
  double average_distance;
  double average_accuracy;
  double covered_roi_volume;
  size_t roi_key_count;
  size_t true_roi_key_count;
  size_t false_roi_key_count;
  std::vector<double> fruit_cell_counts;
  std::vector<double> fruit_cell_percentages;
  std::vector<double> volume_accuracies;
  std::vector<double> distances;
  std::vector<double> volumes;*/

  results.roi_key_count = roi_keys.size();

  // Computations directly with ROI cells
  if (gt_comparison)
  {
    results.fruit_cell_counts.resize(gt_params.num_fruits);
    results.fruit_cell_percentages.resize(gt_params.num_fruits);
    results.volume_accuracies.resize(gt_params.num_fruits);
    results.distances.resize(gt_params.num_fruits);
    roi_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());
    clusters.clear();
    clusters.resize(gt_params.num_fruits);
    for (const octomap::OcTreeKey &key : roi_keys)
    {
      // new per-fruit comparison
      unsigned int ind = gtLoader->getFruitIndex(key);
      if (ind != 0)
      {
        octomap::point3d coord = planner->getPlanningTree()->keyToCoord(key);
        clusters[ind - 1].indices.push_back(roi_pcl->size());
        roi_pcl->push_back(octomap_vpp::octomapPointToPcl<pcl::PointXYZ>(coord));
        results.fruit_cell_counts[ind - 1]++;
        results.true_roi_key_count++;
      }
      else
      {
        results.false_roi_key_count++;
      }
    }

    if (save_pointcloud)
    {
      std::stringstream filename;
      filename << "pointcloud_" << trial_num << "_" << step << ".pcd";
      saveClustersAsColoredCloud(filename.str(), roi_pcl, clusters);
    }

    hulls_clouds.clear();
    hulls_polygons.clear();
    std::vector<pcl::PointXYZ> centroids;
    computeHullsAndVolumes(roi_pcl, clusters, hulls_clouds, hulls_polygons, results.volumes, centroids);

    for (size_t i = 0; i < gt_params.num_fruits; i++)
    {
      if (results.fruit_cell_counts[i] > 0)
      {
        results.detected_roi_clusters++;
        results.fruit_cell_percentages[i] = (double)results.fruit_cell_counts[i] / (double)gtLoader->getNumFruitCells(i);
        results.distances[i] = pcl::squaredEuclideanDistance(gt_centroids[i], centroids[i]);
        results.volume_accuracies[i] = results.volumes[i] / gt_params.fruit_volumes[i];
        results.average_accuracy += results.volume_accuracies[i];
        results.covered_roi_volume += results.volume_accuracies[i];
        results.average_distance += results.distances[i];
      }

    }
    results.average_distance /= results.detected_roi_clusters;
    results.average_accuracy /= results.detected_roi_clusters;
    results.covered_roi_volume /= gt_params.num_fruits;
  }

  results.roi_key_count = roi_keys.size();

  return results;
}

void Evaluator::saveClustersAsColoredCloud(const std::string &filename, pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, const std::vector<pcl::PointIndices> &clusters)
{
  if (!input_cloud || input_cloud->empty())
    return;

  pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
  colored_cloud.reserve(input_cloud->size());
  for (size_t cid=0; cid < clusters.size(); cid++)
  {
    pcl::RGB color = pcl::GlasbeyLUT::at(cid % pcl::GlasbeyLUT::size());
    for (int ind : clusters[cid].indices)
    {
      pcl::PointXYZRGB p(color.r, color.b, color.g);
      p.x = input_cloud->at(ind).x;
      p.y = input_cloud->at(ind).y;
      p.z = input_cloud->at(ind).z;
      colored_cloud.push_back(p);
    }
  }

  if (colored_cloud.empty())
    return;

  pcl::io::savePCDFile(filename, colored_cloud, true);
}

ostream& Evaluator::writeHeader(ostream &os)
{
  os << "Detected ROI cluster,Average distance,Average volume accuracy,Covered ROI volume,ROI key count,True ROI keys,False ROI keys";
  return os;
}

ostream& Evaluator::writeParams(ostream &os, const EvaluationParameters &res)
{
  /*size_t detected_roi_clusters;
  double average_distance;
  double average_accuracy;
  double covered_roi_volume;
  size_t roi_key_count;
  size_t true_roi_key_count;
  size_t false_roi_key_count;
  std::vector<double> fruit_cell_counts;
  std::vector<double> fruit_cell_percentages;
  std::vector<double> volume_accuracies;
  std::vector<double> distances;
  std::vector<double> volumes;*/
  os << res.detected_roi_clusters << "," << res.average_distance << "," << res.average_accuracy << "," << res.covered_roi_volume
     << "," << res.roi_key_count << "," << res.true_roi_key_count << "," << res.false_roi_key_count;
  return os;
}

}
