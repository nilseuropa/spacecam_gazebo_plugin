#include "spacecam_gazebo.cpp"
#include <gazebo/plugins/CameraPlugin.hh>

namespace gazebo
{

  template <>
  void GazeboRosSpaceCam_<CameraPlugin>::LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    this->camera_ = this->CameraPlugin::camera;
  }

  template class GazeboRosSpaceCam_<CameraPlugin>;
  typedef GazeboRosSpaceCam_<CameraPlugin> GazeboRosSpaceCam;

  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosSpaceCam)
}
