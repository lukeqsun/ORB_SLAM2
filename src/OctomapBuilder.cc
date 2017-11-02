#include "OctomapBuilder.h"

namespace ORB_SLAM2 {

OctomapBuilder::OctomapBuilder(const string& strSettingPath) {
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  float res = fSettings["OctomapBuilder.Resolution"];
  globalOctoMap = new octomap::OcTree(res);
  globalOctoMap->setOccupancyThres(0.5);
  // globalOctoMap->setProbMiss(0.51);
  hasUpdate = false;
  camera_cx = fSettings["Camera.cx"];
  camera_cy = fSettings["Camera.cy"];
  camera_fx = fSettings["Camera.fx"];
  camera_fy = fSettings["Camera.fy"];

  cv::Mat T_cb_mat = (cv::Mat_< float >(4, 4) << 0, -1, 0, -0.1,  //-0.1,
                      0, 0, -1, 0, 1, 0, 0, -0.22,                //-0.22,
                      0, 0, 0, 1);

  float df = fSettings["OctomapBuilder.DepthFactor"];
  depthFactor = 1.0 / df;

  T_bc = T_cb_mat.inv();
}

void OctomapBuilder::Run() {
  while (1) {
    unique_lock< mutex > lock(mMutexUpdate);
    cvUpdate.wait(lock, [&] {
      if (this->hasUpdate) {
        this->hasUpdate = false;
        return true;
      } else {
        return false;
      }
    });

    // Update the octomap.
    Eigen::Matrix4f T_wc_eig = Converter::toMatrix4f(T_wc_mat);
    Eigen::Quaternionf q_wc(T_wc_eig.topLeftCorner< 3, 3 >());

    octomap::pose6d T_wc_octo(
        octomap::point3d(T_wc_eig(0, 3), T_wc_eig(1, 3),
                         T_wc_eig(2, 3)),  // for 3d case we need z coordinates
        octomath::Quaternion(q_wc.w(), q_wc.x(), q_wc.y(), q_wc.z()));

    octomap::Pointcloud local_cloud;

    for (int m = 0; m < depth.rows; m++) {
      for (int n = 0; n < depth.cols; n++) {
        float d = depth.at< float >(m, n);

        if (d < 0.01 || d > 1.0) continue;
        float z = d;
        float x = (float(n) - camera_cx) * z / camera_fx;
        float y = (float(m) - camera_cy) * z / camera_fy;

        // if (y > 0.2 || y < -0.25) continue;
        // y=0;
        Eigen::Vector4f hPt;
        hPt << x, y, z, 1;
        Eigen::Vector4f hPt_w = T_wc_eig * hPt;
        local_cloud.push_back(hPt_w(0), hPt_w(1), hPt_w(2));
      }
    }

    // local_cloud.transform(T_wc_octo);
    octomath::Vector3 vec3 = T_wc_octo.trans();
    unique_lock< mutex > lock2(mMutexRequest);
    globalOctoMap->insertPointCloud(
        local_cloud,
        octomap::point3d(vec3.x(), vec3.y(),
                         vec3.z()));  // for 3d case we need z coordinates
    globalOctoMap->updateInnerOccupancy();

    lock2.unlock();
  }
}

void OctomapBuilder::saveOctoMap(const string& filename) {
  unique_lock< mutex > lock2(mMutexRequest);
  if (globalOctoMap) {
    globalOctoMap->writeBinary(filename);
  }
}

void OctomapBuilder::reset() {
  unique_lock< mutex > lock2(mMutexRequest);
  if (globalOctoMap) {
    globalOctoMap->clear();
    OccupiedPoints.clear();
    FreePoints.clear();
  }
}

// This function is called from Tracking thread.
void OctomapBuilder::UpdateOctomap(const cv::Mat& depth_, cv::Mat currPose_) {
  unique_lock< mutex > lock(mMutexUpdate);
  hasUpdate = true;
  depth_.convertTo(depth, CV_32FC1, depthFactor);
  currPose = currPose_.clone();
  T_wc_mat = currPose * T_bc;
  lock.unlock();
  cvUpdate.notify_one();
}

bool OctomapBuilder::calcOccupiedPoints() {
  // vector<vector<float>> occupiedPoints;
  unique_lock< mutex > lock(mMutexRequest);
  OccupiedPoints.clear();
  FreePoints.clear();
  for (auto it = globalOctoMap->begin(); it != globalOctoMap->end(); it++) {
    if (globalOctoMap->isNodeOccupied(*it)) {
      OccupiedPoints.push_back({it.getCoordinate().x(), it.getCoordinate().y(),
                                it.getCoordinate().z()});
    } else {
      FreePoints.push_back({it.getCoordinate().x(), it.getCoordinate().y(),
                            it.getCoordinate().z()});
    }
  }
  lock.unlock();
  return true;
}

vector< vector< float > > OctomapBuilder::getOccupiedPoints() {
  return OccupiedPoints;
}

}  // namespace ORB_SLAM2
