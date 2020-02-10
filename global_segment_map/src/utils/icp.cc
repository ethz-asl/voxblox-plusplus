#include "global_segment_map/utils/icp.h"

#include <glog/logging.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace voxblox {

bool next_iteration = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                           void* nothing) {
  if (event.getKeySym() == "space" && event.keyDown()) next_iteration = true;
}

ICP::ICP() : max_iterations_(5000) {
  icp_.setEuclideanFitnessEpsilon(1e-5);
  icp_.setMaximumIterations(max_iterations_);
  icp_.setTransformationEpsilon(1e-12);
}

void print4x4Matrix(const Eigen::Matrix4d& matrix) {
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1),
         matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1),
         matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1),
         matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3),
         matrix(2, 3));
}

void ICP::align(const PointCloudType::Ptr cloud_in,
                const PointCloudType::Ptr cloud_tr,
                Eigen::Matrix4f* transformation_matrix_float) {
  Eigen::Matrix4d transformation_matrix;
  icp_.setMaximumIterations(max_iterations_);
  icp_.setInputSource(cloud_in);
  icp_.setInputTarget(cloud_tr);
  PointCloudType::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZRGB>);

  icp_.align(*cloud_icp);
  icp_.setMaximumIterations(1);  // We set this variable to 1 for the next time
                                 // we will call .align () function

  transformation_matrix = icp_.getFinalTransformation().cast<double>();
  print4x4Matrix(transformation_matrix);

  std::cout << "has converged: " << icp_.hasConverged()
            << " score:  " << icp_.getFitnessScore() << std::endl;

  // // Visualization
  // pcl::visualization::PCLVisualizer viewer("ICP demo");
  // // Create two vertically separated viewports
  // int v1(0);
  // int v2(1);
  // viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  // viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  //
  // // The color we will be using
  // float bckgr_gray_level = 0.0;  // Black
  // float txt_gray_lvl = 1.0 - bckgr_gray_level;
  //
  // // Original point cloud is white
  // pcl::visualization::PointCloudColorHandlerCustom<PointType>
  // cloud_in_color_h(
  //     cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
  //     (int)255 * txt_gray_lvl);
  // viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
  // viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
  //
  // // Transformed point cloud is green
  // pcl::visualization::PointCloudColorHandlerCustom<PointType>
  // cloud_tr_color_h(
  //     cloud_tr, 20, 180, 20);
  // viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);
  //
  // // ICP aligned point cloud is red
  // pcl::visualization::PointCloudColorHandlerCustom<PointType>
  // cloud_icp_color_h(
  //     cloud_icp, 180, 20, 20);
  // viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
  //
  // // Adding text descriptions in each viewport
  // viewer.addText(
  //     "White: Original point cloud\nGreen: Matrix transformed point cloud",
  //     10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1",
  //     v1);
  // viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud",
  //                10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,
  //                "icp_info_2", v2);
  //
  // std::stringstream ss;
  // ss << max_iterations_;
  // std::string iterations_cnt = "ICP iterations = " + ss.str();
  // viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl,
  //                txt_gray_lvl, "iterations_cnt", v2);
  //
  // // Set background color
  // viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level,
  //                           bckgr_gray_level, v1);
  // viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level,
  //                           bckgr_gray_level, v2);
  //
  // // Set camera position and orientation
  // viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947,
  //                          -0.256907, 0);
  // viewer.setSize(1280, 1024);  // Visualiser window size
  //
  // // Register keyboard callback :
  // viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
  //
  // // Display the visualiser
  // while (!viewer.wasStopped()) {
  //   viewer.spinOnce();
  //
  //   // The user pressed "space" :
  //   if (next_iteration) {
  //     // The Iterative Closest Point algorithm
  //     icp_.align(*cloud_icp);
  //
  //     if (icp_.hasConverged()) {
  //       // printf("\033[11A");  // Go up 11 lines in terminal output.
  //       printf("\nICP has converged, score is %f", icp_.getFitnessScore());
  //       std::cout << "\nICP transformation " << ++max_iterations_
  //                 << " : cloud_icp -> cloud_in" << std::endl;
  //       transformation_matrix *=
  //           icp_.getFinalTransformation()
  //               .cast<double>();  // WARNING /!\ This is not accurate! For
  //                                 // "educational" purpose only!
  //       print4x4Matrix(
  //           transformation_matrix);  // Print the transformation between
  //                                    // original pose and current pose
  //
  //       ss.str("");
  //       ss << max_iterations_;
  //       std::string iterations_cnt = "ICP iterations = " + ss.str();
  //       viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl,
  //                         txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
  //       viewer.updatePointCloud(cloud_icp, cloud_icp_color_h,
  //       "cloud_icp_v2");
  //     } else {
  //     }
  //   }
  //   next_iteration = false;
  // }

  *transformation_matrix_float = transformation_matrix.cast<float>();
}
}  // namespace voxblox
