#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

class DepthToBinsNode : public rclcpp::Node
{
public:
  explicit DepthToBinsNode(const rclcpp::NodeOptions & options)
  : Node("depth_to_bins_node", options)
  {
    this->declare_parameter<int>("num_bins", 8);
    this->declare_parameter<double>("depth_scale", 1.0);
    num_bins_ = this->get_parameter("num_bins").as_int();
    depth_scale_ = this->get_parameter("depth_scale").as_double();

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "~/depth_image", 1,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { depth_callback(msg); });

    publisher_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>("~/avg_inv_depth", num_bins_);

    RCLCPP_INFO(this->get_logger(), "Node initialized with %d bins.", num_bins_);
  }

private:
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // convert the ros image to opencv format
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat depth_image = cv_ptr->image;
    int bin_width = depth_image.cols / num_bins_;
    std::vector<float> inv_depth_sums(num_bins_, 0.0f);
    std::vector<int> valid_pixel_counts(num_bins_, 0);

    // Loop through each pixel once, accumulate inverse depth values and counts
    // per vertical bin
    for (int r = 0; r < depth_image.rows; ++r) {
      for (int c = 0; c < depth_image.cols; ++c) {
        int bin_index = c / bin_width;
        float depth = depth_image.at<float>(r, c);
        if (depth > 0) {  // avoid zero values (no depth information)
          inv_depth_sums[bin_index] += 1.0f / (depth * depth_scale_);
          valid_pixel_counts[bin_index]++;
        }
      }
    }

    // Calculate average inverse depths per bin
    std::vector<float> avg_inv_depths(num_bins_, 0.0f);
    for (int i = 0; i < num_bins_; ++i) {
      if (valid_pixel_counts[i] > 0) {
        avg_inv_depths[i] = inv_depth_sums[i] / valid_pixel_counts[i];
      }
    }

    // publish results
    auto output_msg = std_msgs::msg::Float32MultiArray();
    output_msg.data = avg_inv_depths;
    publisher_->publish(output_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  int num_bins_;
  float depth_scale_;
};

// register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DepthToBinsNode)
