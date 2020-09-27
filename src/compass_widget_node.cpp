
#include <image_widgets/compass_widget_node.h>


using std::placeholders::_1;

namespace image_widgets
{
  using namespace cv;

  CompassWidget::CompassWidget(const rclcpp::NodeOptions &options) :
    Node("compass_widget", options),
    img_paths({"images/compass_background.png", "images/compass_needle.png"}),
    frame_id("compass_widget")
  {

    this->declare_parameter("img_paths", img_paths);
    this->get_parameter("img_paths", img_paths);
    this->get_parameter("frame_id", frame_id);

    for(std::string img_path : img_paths)
    {
      Mat layer_img = imread(img_path, CV_32FC4);

      // check the sizes of the images
      if((layer_img.rows == 0) || (layer_img.cols == 0)){
        RCLCPP_ERROR(this->get_logger(), "layer from '%s' is %dx%dpx", img_path.c_str(), layer_img.rows, layer_img.cols);
        return;
      }
      
      // check the alpha channel is within 0-1
      double min, max;
      std::vector<Mat> layer_channels(4);
      split(layer_img, layer_channels);
      layer_channels[3] = layer_channels[3] / 255.0;
      merge(layer_channels, layer_img);

      Mat layer_alpha = layer_channels[3];
      minMaxLoc(layer_alpha, &min, &max);
      if(min<0) RCLCPP_WARN(this->get_logger(), "layer from '%s' has alpha channel < 0", img_path.c_str());
      if(max>1) RCLCPP_WARN(this->get_logger(), "layer from '%s' has alpha channel > 1", img_path.c_str());

      // XXX should crash out on the above checks
      img_layers.push_back(layer_img);
    }


    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("compass_image_raw", 2);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "compass_pose", 10, std::bind(&CompassWidget::render_callback, this, _1));
  }

  void CompassWidget::render_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
  {
    // XXX check 'new' for style
    sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

    RCLCPP_INFO(this->get_logger(), "Publishing");

    // XXX think about a mixer table with translations
    float theta = twist_msg->angular.z;

    //friendly names for the layers
    Mat background_img = img_layers[0];
    Mat foreground_img = img_layers[1];

    // compute a transformation for the foreground image
    Mat foreground_rot;
    Point2f pc(foreground_img.cols/2., foreground_img.rows/2.);
    Mat r = getRotationMatrix2D(pc, theta, 1.0);
    warpAffine(foreground_img, foreground_rot, r, background_img.size());
    // XXX deal with foreground items that are smaller than the background

    // split the images into planes
    std::vector<Mat> bg_ch(4);
    std::vector<Mat> fg_ch(4);
    std::vector<Mat> out_ch(4);
    split(background_img, bg_ch);
    split(foreground_rot, fg_ch);

    Mat bg_alpha = bg_ch[3];
    Mat fg_alpha = fg_ch[3];

    Mat output_img = Mat::zeros(background_img.size(), background_img.type());
    split(output_img, out_ch);

    // overlay foreground onto background
    for(int i=0; i<3; i++)
    {
      //out_ch[i] = (fg_alpha * fg_ch[i]) + ((1.0-fg_alpha) * (bg_alpha * bg_ch[i]));
      out_ch[i] = fg_alpha.mul(fg_ch[i]) + ((1.0-fg_alpha).mul(bg_ch[i]));
    }
    out_ch[3] = fg_alpha + ((1.0-fg_alpha).mul(bg_alpha));
    merge(out_ch, output_img); // not necessary? the above channelwise ops should be in-place

    // XXX use stamped twist message and carry the stamp forward
    auto stamp = now();
    //auto stamp = twist_msg->header.stamp;
    
    // Convert OpenCV Mat to ROS Image
    image_msg->header.stamp = stamp;
    image_msg->header.frame_id = frame_id;
    image_msg->height = output_img.rows;
    image_msg->width = output_img.cols;
    image_msg->encoding = mat_type2encoding(output_img.type());
    image_msg->is_bigendian = false;
    image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(output_img.step);
    image_msg->data.assign(output_img.datastart, output_img.dataend);

    // Publish

    publisher_->publish(std::move(image_msg));
  }

} //namespace image_widgets



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options{};
    rclcpp::spin(std::make_shared<image_widgets::CompassWidget>(options));
    rclcpp::shutdown();
    return 0;
}
