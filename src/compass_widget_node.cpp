
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
      Mat layer_img = imread(img_path, CV_8UC4);  //prefer 8b png
      layer_img.convertTo(layer_img, CV_32FC4, 1/255.0);  //convert to float with range 1-0

      // check the sizes of the images
      if((layer_img.rows == 0) || (layer_img.cols == 0)){
        RCLCPP_ERROR(this->get_logger(), "layer from '%s' is %dx%dpx", img_path.c_str(), layer_img.rows, layer_img.cols);
        return;
      }
      
      // check the alpha channel is within 0-1
      double min, max;
      std::vector<Mat> layer_channels(4);
      split(layer_img, layer_channels);
      
      Mat layer_alpha = layer_channels[3];
      minMaxLoc(layer_alpha, &min, &max);
      if(min<0) RCLCPP_WARN(this->get_logger(), "layer from '%s' has alpha channel < 0", img_path.c_str());
      if(max>1) RCLCPP_WARN(this->get_logger(), "layer from '%s' has alpha channel > 1", img_path.c_str());

      img_layers.push_back(layer_img);
    }


    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("compass_image_raw", 2);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "compass_pose", 10, std::bind(&CompassWidget::render_callback, this, _1));
  }

  void CompassWidget::render_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
  {
    // XXX think about a mixer table with translations
    // XXX deal with foreground items that are smaller than the background
    // XXX use stamped message and carry the stamp forward

    //friendly names for the layers
    float theta = twist_msg->angular.z;
    Mat background_img = img_layers[0];
    Mat foreground_img = img_layers[1];
    Mat output_img = draw_dial(foreground_img, background_img, theta);


    output_img.convertTo(output_img, CV_8UC4, 255.0);

    sensor_msgs::msg::Image image_msg; // >> message to be sent
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::RGBA8, output_img);
    img_bridge.toImageMsg(image_msg); // from cv_bridge to sensor_msgs::Image

    image_msg.header.stamp = now();
    image_msg.header.frame_id = frame_id;

    publisher_->publish(image_msg);
    //publisher_->publish(img_bridge.toImageMsg());
  }

  Mat draw_dial(Mat foreground_img, Mat background_img, float theta)
  {
    Mat foreground_rot = rotate_image_middle(foreground_img, theta);
    Mat output_img = alpha_blending(foreground_rot, background_img);
    return output_img;
  }


  Mat alpha_blending(Mat foreground_img, Mat background_img)
  {
    // split the images into planes
    std::vector<Mat> bg_ch(4);
    std::vector<Mat> fg_ch(4);
    std::vector<Mat> out_ch(4);

    split(background_img, bg_ch);
    split(foreground_img, fg_ch);

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
    return output_img;
  }

  Mat rotate_image_middle(Mat img_src, float angle)
  {
    Mat img_dst;
    Point2f rot_axis(img_src.cols/2., img_src.rows/2.);
    Mat r = getRotationMatrix2D(rot_axis, angle, 1.0);
    warpAffine(img_src, img_dst, r, img_src.size());
    return img_dst;
  }



} //namespace image_widgets

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options{};
    rclcpp::spin(std::make_shared<image_widgets::CompassWidget>(options));
    rclcpp::shutdown();
    return 0;
}

