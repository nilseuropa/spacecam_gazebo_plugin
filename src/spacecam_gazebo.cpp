#include <spacecam_gazebo/spacecam_gazebo.h>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <sensor_msgs/image_encodings.h>
#include <gazebo/gazebo_config.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <vector>

namespace gazebo
  {

  template <class Base> // constructor
  GazeboRosSpaceCam_<Base>::GazeboRosSpaceCam_(){}

  template <class Base> // destructor
  GazeboRosSpaceCam_<Base>::~GazeboRosSpaceCam_(){}

  template <class Base> // copy of cameraPlugin
  void GazeboRosSpaceCam_<Base>::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {

    Base::Load(_parent, _sdf);
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;

    this->image_connect_count_ = boost::shared_ptr<int>(new int);
    *this->image_connect_count_ = 0;
    this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
    this->was_active_ = boost::shared_ptr<bool>(new bool);
    *this->was_active_ = false;

    // this->blob_pub_ = nh_.advertise<std_msgs::UInt16MultiArray>("/spacecam/blobs", 100);

    // this->params_.filterByArea = false;
    // this->params_.filterByColor = true;
    // this->params_.blobColor = 255;

    LoadImpl(_parent, _sdf);
    GazeboRosCameraUtils::Load(_parent, _sdf);
    ROS_WARN("SpaceCam plugin has started.");
  }

  template <class Base> // update
  void GazeboRosSpaceCam_<Base>::OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format) {
    if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
      return;

    #if (GAZEBO_MAJOR_VERSION > 6)
      this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
    #else
      this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
    #endif

    if (!this->parentSensor->IsActive()) {
      if ((*this->image_connect_count_) > 0) this->parentSensor->SetActive(true);
    }
    else {
      if ((*this->image_connect_count_) > 0) {
        #if (GAZEBO_MAJOR_VERSION >= 8)
              common::Time cur_time = this->world_->SimTime();
        #else
              common::Time cur_time = this->world_->GetSimTime();
        #endif
        if (cur_time - this->last_update_time_ >= this->update_period_) {
          this->PutCameraData(_image);
          this->PublishCameraInfo();
          this->last_update_time_ = cur_time;
        }
      }
    }
  }

  template <class Base>
  void GazeboRosSpaceCam_<Base>::OnNewImageFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format) {
    OnNewFrame(_image, _width, _height, _depth, _format);
  }

  template <class Base> // load data to interface
  void GazeboRosSpaceCam_<Base>::PutCameraData(const unsigned char *_src, common::Time &last_update_time)
  {
    this->sensor_update_time_ = last_update_time;
    this->PutCameraData(_src);
  }

  template <class Base>
  void GazeboRosSpaceCam_<Base>::PutCameraData(const unsigned char *_src)
  {
    if (!this->initialized_ || this->height_ <=0 || this->width_ <=0) return;

    this->lock_.lock();

    // copy data into ROS image
    this->image_msg_.header.frame_id = this->frame_name_;
    this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
    this->image_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

    if ((*this->image_connect_count_) > 0) {

      this->image_msg_.width = this->width_;
      this->image_msg_.height = this->height_;
      this->image_msg_.encoding = sensor_msgs::image_encodings::MONO8; // actually black and white, but close enough
      this->image_msg_.step = this->image_msg_.width;

      size_t size = this->width_ * this->height_;

      std::vector<uint8_t>& data (this->image_msg_.data);
      data.resize(size);

      size_t img_index = 0;

      for (size_t i = 0; i < size; ++i){
        if ((_src[img_index] >254) && (_src[img_index+1] < 1) && (_src[img_index+2] < 1)) { // if glowing RED
          data[i] = 255; // white
        }
        else {
          data[i] = 0; // everything else is black
        }
        img_index += 3;
      }

      // this->blob_pub_.publish(this->blobArray_);
      this->image_pub_.publish(this->image_msg_);
    }

    this->lock_.unlock();
  }
}
