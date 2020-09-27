#ifndef IMAGE_WIDGETS_HPP
#define IMAGE_WIDGETS_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>

//#include <opencv/cv.h>
//#include <opencv2/imgcodecs.hpp>
//#include cv geometry
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


using std::placeholders::_1;

namespace image_widgets
{
  using namespace cv;


  // thanks to opencv_cam
  std::string mat_type2encoding(int mat_type)
  {
    switch (mat_type)
    {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("unsupported encoding type");
    }
  }

  class CompassWidget : public rclcpp::Node
  {
    public:

      CompassWidget(const rclcpp::NodeOptions &options);

    private:
      void render_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

      std::vector<std::string> img_paths;
      std::vector<Mat> img_layers;
      std::string frame_id;
  };

} // namespace image_widgets

#endif //IMAGE_WIDGETS_HPP
