/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2 {

Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer,
               MapDrawer *pMapDrawer, Tracking *pTracking,
               const string &strSettingPath, bool mbReuseMap_)
    : mpSystem(pSystem),
      mpFrameDrawer(pFrameDrawer),
      mpMapDrawer(pMapDrawer),
      mpTracker(pTracking),
      mbFinishRequested(false),
      mbFinished(true),
      mbFlipCamera(-1.0),
      mbRequestQuit(false),
      mbStopped(true),
      mbStopRequested(false),
      mbReuseMap(mbReuseMap_) {
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

  float fps = fSettings["Camera.fps"];
  if (fps < 1) fps = 30;
  // mT = 1e3 / fps;

  mImageWidth = fSettings["Camera.width"];
  mImageHeight = fSettings["Camera.height"];
  if (mImageWidth < 1 || mImageHeight < 1) {
    mImageWidth = 480;
    mImageHeight = 270;
  } else if (mImageWidth > 600 || mImageHeight > 600) {
    mImageWidth /= 2;
    mImageHeight /= 2;
  }

  mbFlipCamera = fSettings["Camera.flip"];

  mViewpointX = fSettings["Viewer.ViewpointX"];
  mViewpointY = fSettings["Viewer.ViewpointY"];
  mViewpointZ = fSettings["Viewer.ViewpointZ"];
  mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer::Run() {
  mbFinished = false;
  mbStopped = false;

  float width = 1920;
  float height = 600;

  float modelWidth = 1024;
  float modelHeight = 768;

  float frameWidth = mImageWidth;
  float frameHeight = mImageHeight;

  float panelWidth = 200;

  pangolin::CreateWindowAndBind("ORB-SLAM2: Main", width, height);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu0");
  pangolin::CreatePanel("menu1");
  pangolin::CreatePanel("menu2");
  // pangolin::CreatePanel("menu3");
  pangolin::CreatePanel("menu4");
  pangolin::CreatePanel("menu5");

  pangolin::Var< bool > menuShowPoints("menu0.Show Points", true, true);
  pangolin::Var< bool > menuShowOctomap("menu0.Show Octomap", true, false);
  pangolin::Var< bool > menuShowKeyFrames("menu0.Show KeyFrames", true, true);
  pangolin::Var< bool > menuFollowCamera("menu1.Follow Camera", true, false);
  pangolin::Var< bool > menuShowGraph("menu1.Show Graph", true, true);
  pangolin::Var< bool > menuLocalizationMode("menu1.Localization Mode",
                                             mbReuseMap, false);
  pangolin::Var< std::string > menuState("menu2.State", "Unknown");
  pangolin::Var< bool > menuSaveMap("menu2.Save Map", false, false);

  pangolin::Var< bool > menuReset("menu4.Reset", false, false);
  pangolin::Var< std::string > menuKFs("menu4.KFs", "0");
  pangolin::Var< std::string > menuMPs("menu4.MPs", "0");
  pangolin::Var< bool > menuQuit("menu5.Quit", false, false);
  pangolin::Var< std::string > menuMatches("menu5.Matches", "0");
  pangolin::Var< std::string > menuVmatches("menu5.+ VO matches", "0");

  pangolin::Display("menu")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(panelWidth))
      .SetLayout(pangolin::LayoutEqualVertical)
      .AddDisplay(pangolin::Display("menu0"))
      .AddDisplay(pangolin::Display("menu1"))
      .AddDisplay(pangolin::Display("menu2"))
      // .AddDisplay(pangolin::Display("menu3"))
      .AddDisplay(pangolin::Display("menu4"))
      .AddDisplay(pangolin::Display("menu5"));

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(modelWidth, modelHeight, mViewpointF,
                                 mViewpointF, modelWidth / 2.0,
                                 modelHeight / 2.0, 0.1, 1000),
      pangolin::ModelViewLookAt(mViewpointX, -mbFlipCamera * mViewpointY,
                                mViewpointZ, 0, 0, 0, 0.0, mbFlipCamera * 1.0,
                                0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &d_cam = pangolin::Display("Model")
                              .SetAspect(-modelWidth / modelHeight)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::GlTexture imageTexture(frameWidth, frameHeight, GL_RGB, false, 0,
                                   GL_RGB, GL_UNSIGNED_BYTE);

  pangolin::Display("Frame").SetAspect(frameWidth / frameHeight);
  pangolin::Display("multi")
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(panelWidth), 1.0)
      .SetLayout(pangolin::LayoutEqual)
      .AddDisplay(d_cam)
      .AddDisplay(pangolin::Display("Frame"));

  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  bool bFollow = true;
  bool bLocalizationMode = false;

  while (1) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

    if (menuFollowCamera && bFollow) {
      s_cam.Follow(Twc);
    } else if (menuFollowCamera && !bFollow) {
      // gluLookAt
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
          mViewpointX, -mbFlipCamera * mViewpointY, mViewpointZ, 0, 0, 0, 0.0,
          mbFlipCamera * 1.0, 0.0));
      s_cam.Follow(Twc);
      bFollow = true;
    } else if (!menuFollowCamera && bFollow) {
      bFollow = false;
    }

    if (menuLocalizationMode && !bLocalizationMode) {
      mpSystem->ActivateLocalizationMode();
      bLocalizationMode = true;
    } else if (!menuLocalizationMode && bLocalizationMode) {
      mpSystem->DeactivateLocalizationMode();
      bLocalizationMode = false;
    }

    std::string info = mpFrameDrawer->GetTextInfo();
    std::string delimiter = mpFrameDrawer->mInfoDelimiter;
    size_t pos = 0;
    std::string token;
    int n = 0;
    while ((pos = info.find(delimiter)) != std::string::npos) {
      token = info.substr(0, pos);
      switch (n) {
        case 0:
          menuState.operator=(token);
          break;
        case 1:
          menuKFs.operator=(token);
          break;
        case 2:
          menuMPs.operator=(token);
          break;
        case 3:
          menuMatches.operator=(token);
          menuVmatches.operator=("0");
          break;
        case 4:
          menuVmatches.operator=(token);
          break;
        default:
          menuState.operator=("Unknown");
      }
      n++;

      info.erase(0, pos + delimiter.length());
    }

    cv::Mat img = mpFrameDrawer->DrawFrame();
    cv::flip(img, img, 1);
    cv::resize(img, img, cv::Size(frameWidth, frameHeight));
    imageTexture.Upload((uchar *)img.data, GL_RGB, GL_UNSIGNED_BYTE);

    pangolin::Display("Frame").Activate();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    imageTexture.RenderToViewport();

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    mpMapDrawer->DrawCurrentCamera(Twc);
    if (menuShowKeyFrames || menuShowGraph)
      mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
    if (menuShowPoints) mpMapDrawer->DrawMapPoints();

    if (menuShowOctomap) mpMapDrawer->DrawMapCollision();

    pangolin::FinishFrame();

    if (menuSaveMap) {
      mpSystem->SaveMap();
      menuSaveMap = false;
    }

    if (menuReset) {
      menuShowGraph = true;
      menuShowKeyFrames = true;
      menuShowPoints = true;
      menuShowOctomap = true;
      menuLocalizationMode = false;
      if (bLocalizationMode) mpSystem->DeactivateLocalizationMode();
      bLocalizationMode = false;
      bFollow = true;
      menuFollowCamera = true;
      mpSystem->Reset();
      menuReset = false;
    }

    if (menuQuit) {
      mbRequestQuit = true;
    }

    if (Stop()) {
      while (isStopped()) {
        std::this_thread::sleep_for(std::chrono::microseconds(3000));
      }
    }

    if (CheckFinish()) break;
  }

  SetFinish();
}

void Viewer::RequestFinish() {
  unique_lock< mutex > lock(mMutexFinish);
  mbFinishRequested = true;
}

bool Viewer::CheckFinish() {
  unique_lock< mutex > lock(mMutexFinish);
  return mbFinishRequested;
}

void Viewer::SetFinish() {
  unique_lock< mutex > lock(mMutexFinish);
  mbFinished = true;
}

bool Viewer::isFinished() {
  unique_lock< mutex > lock(mMutexFinish);
  return mbFinished;
}

bool Viewer::shouldFinished() {
  // return pangolin::ShouldQuit();
  return mbRequestQuit;
}

void Viewer::RequestStop() {
  unique_lock< mutex > lock(mMutexStop);
  if (!mbStopped) mbStopRequested = true;
}

bool Viewer::isStopped() {
  unique_lock< mutex > lock(mMutexStop);
  return mbStopped;
}

bool Viewer::Stop() {
  unique_lock< mutex > lock(mMutexStop);
  unique_lock< mutex > lock2(mMutexFinish);

  if (mbFinishRequested)
    return false;
  else if (mbStopRequested) {
    mbStopped = true;
    mbStopRequested = false;
    return true;
  }

  return false;
}

void Viewer::Release() {
  unique_lock< mutex > lock(mMutexStop);
  mbStopped = false;
}

}  // namespace ORB_SLAM2
