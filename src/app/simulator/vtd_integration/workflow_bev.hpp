#pragma once

#include "app/simulator/vtd_integration/vtd_dependencies/pattern/workflow.h"

#include "app/simulator/vtd_integration/bev_detected/frontwide_camera_detected.h"
#include "app/simulator/vtd_integration/bev_detected/frontright_camera_detected.h"
#include "app/simulator/vtd_integration/bev_detected/frontleft_camera_detected.h"
#include "app/simulator/vtd_integration/bev_detected/rear_camera_detected.h"
#include "app/simulator/vtd_integration/bev_detected/rearright_camera_detected.h"
#include "app/simulator/vtd_integration/bev_detected/rearleft_camera_detected.h"

namespace smartsil::app::vtd_integration {
class WorkflowLoader {
 public:
  WorkflowLoader(const std::string& deployment) : deployment_(deployment) {}

  void init(Workflow* wf) {
    front_wide_camera_detected_  = std::make_shared<FrontWideCameraDetected>("bev front wide", "BFW:ALL", "BEV");
    front_right_camera_detected_ = std::make_shared<FrontRightCameraDetected>("bev front right", "BFR:ALL", "BEV");
    front_left_camera_detected_  = std::make_shared<FrontLeftCameraDetected>("bev front left", "BFL:ALL", "BEV");
	
  if (deployment_.find("BFW") != std::string::npos ||
	  deployment_.find("ALL") != std::string::npos) {
	wf->addMethod(front_wide_camera_detected_.get(), deployment_);

  }
  
  if (deployment_.find("BFR") != std::string::npos ||
	  deployment_.find("ALL") != std::string::npos) {
	wf->addMethod(front_right_camera_detected_.get(), deployment_);

  }
  
  if (deployment_.find("BFL") != std::string::npos ||
	  deployment_.find("ALL") != std::string::npos) {
	wf->addMethod(front_left_camera_detected_.get(), deployment_);

  }

 private:
  std::string deployment_;
  std::shared_ptr<FrontWideCameraDetected> front_wide_camera_detected_;
  std::shared_ptr<FrontRightCameraDetected> front_right_camera_detected_;
  std::shared_ptr<FrontLeftCameraDetected> front_left_camera_detected_;
  std::shared_ptr<RearCameraDetected> rear_camera_detected_;
  std::shared_ptr<RearRightCameraDetected> rear_right_camera_detected_;
  std::shared_ptr<RearLeftCameraDetected> rear_left_camera_detected_;
};
} // namespace smartsil::app::vtd_integration
