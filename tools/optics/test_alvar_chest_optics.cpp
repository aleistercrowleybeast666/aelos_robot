// Run against the separately built candidate library on an isolated ROS master.
// This executable supplies synthetic image corners and publishes no topics.
#include <ar_track_alvar/Camera.h>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_track_alvar");
  ros::NodeHandle node, private_node("~");
  bool expect_distortion = false;
  if (!private_node.getParam("expect_distortion", expect_distortion))
    return 2;
  const cv::Mat K = (cv::Mat_<double>(3,3) << 228.482, 0., 318.591, 0., 229.544, 236.217, 0., 0., 1.);
  sensor_msgs::CameraInfo info;
  info.header.frame_id = "chest_usb_cam";
  info.width = 640;
  info.height = 480;
  info.distortion_model = "plumb_bob";
  std::copy(K.ptr<double>(), K.ptr<double>() + 9, info.K.begin());
  // Deliberately nonzero k3 to catch truncation to ALVAR's four-coefficient cache.
  info.D = {0.1001, -0.0598, -0.00326, 0.000126, 0.018};
  const bool edge_pixels = private_node.param("edge_pixels", false);
  if (edge_pixels)
    info.D[4] = 0.; // AELOS chest model, whose extreme corner is not invertible.
  alvar::Camera camera(node, "camera_info");
  camera.SetCameraInfo(info);
  const cv::Mat rotation = (cv::Mat_<double>(3,1) << 0.7, -0.24, 0.11);
  const cv::Mat translation = (cv::Mat_<double>(3,1) << -0.23, 0.03, 0.30);
  std::vector<cv::Point3d> object = {{-.022,-.022,0.}, {.022,-.022,0.},
                                     {.022,.022,0.}, {-.022,.022,0.}};
  std::vector<cv::Point2d> raw;
  cv::projectPoints(object, rotation, translation, K, info.D, raw);
  if (edge_pixels)
    raw = {{4.,4.}, {34.,8.}, {32.,32.}, {5.,29.}};
  std::vector<alvar::PointDouble> measured, planar;
  for (size_t i = 0; i < raw.size(); ++i)
  {
    measured.emplace_back(raw[i].x, raw[i].y);
    planar.emplace_back(object[i].x, object[i].y);
  }
  cv::Mat reference_r, reference_t;
  cv::solvePnP(object, raw, K, expect_distortion ? cv::Mat(info.D) : cv::Mat(),
               reference_r, reference_t, false, cv::SOLVEPNP_ITERATIVE);
  // Same planar/pixel/Pose overload as Marker::UpdatePose. This also calls
  // the intermediate planar overload and the single final 3D solver.
  alvar::Pose pose;
  camera.CalcExteriorOrientation(planar, measured, &pose);
  cv::Mat actual_r = cv::Mat::zeros(3,1,CV_64F);
  pose.GetRodriques(actual_r);
  const cv::Mat actual_t(3,1,CV_64F,pose.translation);
  const double r_error = cv::norm(reference_r - actual_r);
  const double t_error = cv::norm(reference_t - actual_t);
  std::cout << "rotation vector difference " << r_error
            << ", translation difference " << t_error << '\n';
  return r_error < 1e-6 && t_error < 1e-7 ? 0 : 1;
}
