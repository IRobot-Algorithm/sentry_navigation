// 2024-7-2 Wu

#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#define inverse

#ifdef USE_VGICP_CUDA
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

// benchmark for PCL's registration methods
template <typename Registration>
void test_pcl(Registration& reg, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);

  double fitness_score = 0.0;

  // single run
  auto t1 = std::chrono::high_resolution_clock::now();
  reg.setInputTarget(target);
  reg.setInputSource(source);
  reg.align(*aligned);
  auto t2 = std::chrono::high_resolution_clock::now();
  fitness_score = reg.getFitnessScore();
  double cost = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
  std::cout << "cost:" << cost << "[msec] " << std::flush;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  // 将目标点云添加到 colored_cloud，设置为红色
  for (const auto& point : *target) {
      pcl::PointXYZRGB colored_point;
      colored_point.x = point.x;
      colored_point.y = point.y;
      colored_point.z = point.z;
      colored_point.r = 255;
      colored_point.g = 0;
      colored_point.b = 0;
      colored_cloud->push_back(colored_point);
  }

  // 将配准后的点云添加到 colored_cloud，设置为绿色
  for (const auto& point : *aligned) {
      pcl::PointXYZRGB colored_point;
      colored_point.x = point.x;
      colored_point.y = point.y;
      colored_point.z = point.z;
      colored_point.r = 0;
      colored_point.g = 255;
      colored_point.b = 0;
      colored_cloud->push_back(colored_point);
  }

  // 保存点云到 PCD 文件
  pcl::io::savePCDFileASCII("colored_aligned.pcd", *colored_cloud);
  pcl::io::savePCDFileASCII("aligned.pcd", *aligned);
}

// benchmark for fast_gicp registration methods
template <typename Registration>
void test(Registration& reg, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);

  double fitness_score = 0.0;

  // single run
  auto t1 = std::chrono::high_resolution_clock::now();
  // fast_gicp reuses calculated covariances if an input cloud is the same as the previous one
  // to prevent this for benchmarking, force clear source and target clouds
  reg.clearTarget();
  reg.clearSource();
  reg.setInputTarget(target);
  reg.setInputSource(source);
  reg.align(*aligned);
  auto t2 = std::chrono::high_resolution_clock::now();
  fitness_score = reg.getFitnessScore();
  double cost = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
  std::cout << "cost:" << cost << "[msec] " << std::flush;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  // 将目标点云添加到 colored_cloud，设置为红色
  for (const auto& point : *target) {
      pcl::PointXYZRGB colored_point;
      colored_point.x = point.x;
      colored_point.y = point.y;
      colored_point.z = point.z;
      colored_point.r = 255;
      colored_point.g = 0;
      colored_point.b = 0;
      colored_cloud->push_back(colored_point);
  }

  // 将配准后的点云添加到 colored_cloud，设置为绿色
  for (const auto& point : *aligned) {
      pcl::PointXYZRGB colored_point;
      colored_point.x = point.x;
      colored_point.y = point.y;
      colored_point.z = point.z;
      colored_point.r = 0;
      colored_point.g = 255;
      colored_point.b = 0;
      colored_cloud->push_back(colored_point);
  }

  // 保存点云到 PCD 文件
  pcl::io::savePCDFileASCII("colored_aligned.pcd", *colored_cloud);
  pcl::io::savePCDFileASCII("aligned.pcd", *aligned);

}

/**
 * @brief main
 */
int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "usage: gicp_align target_pcd source_pcd" << std::endl;
    return 0;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  if (pcl::io::loadPCDFile(argv[1], *target_cloud)) {
    std::cerr << "failed to open " << argv[1] << std::endl;
    return 1;
  }
  if (pcl::io::loadPCDFile(argv[2], *source_cloud)) {
    std::cerr << "failed to open " << argv[2] << std::endl;
    return 1;
  }

  // remove invalid points around origin
  source_cloud->erase(
    std::remove_if(source_cloud->begin(), source_cloud->end(), [=](const pcl::PointXYZ& pt) { return (pt.getVector3fMap().squaredNorm() < 1e-3 ||
                                                                                                      pt.getVector3fMap()(2) > 3.0 ||
                                                                                                      fabs(pt.getVector3fMap()(1)) > 7.5) ||
                                                                                                      pt.getVector3fMap()(0) > 21.0 ||
                                                                                                      pt.getVector3fMap()(0) < -8.0; }),
    source_cloud->end());
  target_cloud->erase(
    std::remove_if(target_cloud->begin(), target_cloud->end(), [=](const pcl::PointXYZ& pt) { return (pt.getVector3fMap().squaredNorm() < 1e-3 ||
                                                                                                      pt.getVector3fMap()(2) < -0.35); }),
    target_cloud->end());

  // downsampling
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.5f, 0.5f, 0.5f);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*filtered);
  target_cloud = filtered;

  filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*filtered);
  source_cloud = filtered;

  std::cout << "target:" << target_cloud->size() << "[pts] source:" << source_cloud->size() << "[pts]" << std::endl;

#ifdef inverse
  // 定义旋转矩阵
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
  float theta = M_PI;
  rotation(0, 0) = cos(theta);
  rotation(0, 1) = -sin(theta);
  rotation(1, 0) = sin(theta);
  rotation(1, 1) = cos(theta);

  // 定义平移向量
  Eigen::Vector3f translation(15.0, 0.0, 0.0);

  // 创建4x4的变换矩阵
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = translation;

  // 对点云应用变换
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform);
  *source_cloud = *transformed_cloud;
#endif

  // std::cout << "--- pcl_gicp ---" << std::endl;
  // pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> pcl_gicp;
  // test_pcl(pcl_gicp, target_cloud, source_cloud);

  // std::cout << "--- pcl_ndt ---" << std::endl;
  // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> pcl_ndt;
  // pcl_ndt.setResolution(1.0);
  // test_pcl(pcl_ndt, target_cloud, source_cloud);

  // std::cout << "--- fgicp_st ---" << std::endl;
  // fast_gicp::FastGICPSingleThread<pcl::PointXYZ, pcl::PointXYZ> fgicp_st;
  // test(fgicp_st, target_cloud, source_cloud);

  std::cout << "--- fgicp_mt ---" << std::endl;
  fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> fgicp_mt;
  // fast_gicp uses all the CPU cores by default
  // fgicp_mt.setNumThreads(8);
  // fgicp_mt.setMaxCorrespondenceDistance(1.0);
  test(fgicp_mt, target_cloud, source_cloud);

  source_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(argv[2], *source_cloud)) {
    std::cerr << "failed to open " << argv[2] << std::endl;
    return 1;
  }

  filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*filtered);

#ifdef inverse
  transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  // 对点云应用变换
  pcl::transformPointCloud(*filtered, *transformed_cloud, transform);
  *filtered = *transformed_cloud;
#endif

  Eigen::Matrix4f final_transform = fgicp_mt.getFinalTransformation();
  // 对点云应用变换
  pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*filtered, *final, final_transform);

  pcl::io::savePCDFileASCII("final.pcd", *final);


  // std::cout << "--- vgicp_st ---" << std::endl;
  // fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp;
  // vgicp.setResolution(1.0);
  // vgicp.setNumThreads(1);
  // test(vgicp, target_cloud, source_cloud);

  // std::cout << "--- vgicp_mt ---" << std::endl;
  // vgicp.setNumThreads(omp_get_max_threads());
  // test(vgicp, target_cloud, source_cloud);

#ifdef USE_VGICP_CUDA
  std::cout << "--- ndt_cuda (P2D) ---" << std::endl;
  fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ> ndt_cuda;
  ndt_cuda.setResolution(1.0);
  ndt_cuda.setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
  test(ndt_cuda, target_cloud, source_cloud);

  std::cout << "--- ndt_cuda (D2D) ---" << std::endl;
  ndt_cuda.setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
  test(ndt_cuda, target_cloud, source_cloud);

  std::cout << "--- vgicp_cuda (parallel_kdtree) ---" << std::endl;
  fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp_cuda;
  vgicp_cuda.setResolution(1.0);
  // vgicp_cuda uses CPU-based parallel KDTree in covariance estimation by default
  // on a modern CPU, it is faster than GPU_BRUTEFORCE
  // vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::CPU_PARALLEL_KDTREE);
  test(vgicp_cuda, target_cloud, source_cloud);

  std::cout << "--- vgicp_cuda (gpu_bruteforce) ---" << std::endl;
  // use GPU-based bruteforce nearest neighbor search for covariance estimation
  // this would be a good choice if your PC has a weak CPU and a strong GPU (e.g., NVIDIA Jetson)
  vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_BRUTEFORCE);
  test(vgicp_cuda, target_cloud, source_cloud);

  std::cout << "--- vgicp_cuda (gpu_rbf_kernel) ---" << std::endl;
  // use RBF-kernel-based covariance estimation
  // extremely fast but maybe a bit inaccurate
  vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
  // kernel width (and distance threshold) need to be tuned
  vgicp_cuda.setKernelWidth(0.5);
  test(vgicp_cuda, target_cloud, source_cloud);
#endif

  return 0;
}
