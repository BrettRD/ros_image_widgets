#ifndef IMAGE_WIDGETS_HPP
#define IMAGE_WIDGETS_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <opencv/cv.h>
//#include <opencv2/imgcodecs.hpp>
//#include cv geometry
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


using std::placeholders::_1;

namespace image_widgets
{
  using namespace cv;


  Mat draw_dial(Mat foreground_img, Mat background_img, float theta);
  Mat alpha_blending(Mat foreground_img, Mat background_img);
  Mat rotate_image_middle(Mat img_src, float angle);

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
