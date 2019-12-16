#ifndef SPACECAM_GAZEBO_HH
#define SPACECAM_GAZEBO_HH

#include <gazebo_plugins/gazebo_ros_camera_utils.h>
// #include <std_msgs/UInt16MultiArray.h>
// #include <opencv2/features2d.hpp>

namespace gazebo
{
  template <typename Base>
  class GazeboRosSpaceCam_ : public Base, GazeboRosCameraUtils
  {
    public: GazeboRosSpaceCam_();
    public: ~GazeboRosSpaceCam_();
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    public: void LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {}

    protected: virtual void OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    protected: virtual void OnNewImageFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    protected: void PutCameraData(const unsigned char *_src);
    protected: void PutCameraData(const unsigned char *_src, common::Time &last_update_time);

    // ros::NodeHandle nh_;
    // ros::Publisher blob_pub_;
    // std_msgs::UInt16MultiArray blobArray_;
    // cv::SimpleBlobDetector::Params params_;
  };

}
#endif
