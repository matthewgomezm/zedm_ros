// Copyright 2025 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::placeholders;
using namespace std;

class MinimalDepthSubscriber : public rclcpp::Node
{
public:
  MinimalDepthSubscriber()
  : Node("ORL_node")
  {
    /* Note: it is very important to use a QOS profile for the subscriber that
     * is compatible with the QOS profile of the publisher. The ZED component
     * node uses a default QoS profile with reliability set as "RELIABLE" and
     * durability set as "VOLATILE". To be able to receive the subscribed topic
     * the subscriber must use compatible parameters.
     */

    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

    rclcpp::QoS depth_qos(10);

    auto sub_opt = rclcpp::SubscriptionOptions();

    // Create depth map subscriber
    mDepthSub = create_subscription<sensor_msgs::msg::Image>(
      "depth", depth_qos,
      std::bind(&MinimalDepthSubscriber::depthCallback, this, _1), sub_opt);
  }

protected:
void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Reinterpret raw bytes as floats
    float * depths = reinterpret_cast<float *>(&msg->data[0]);

    int width = msg->width;
    int height = msg->height;

    // Center coordinates
    int center_u = width / 2;
    int center_v = height / 2;

    // We want a 5x5 matrix, so we look at 2 pixels in every direction
    int offset = 2; 

    std::stringstream ss;
    ss << "\n--- 5x5 Depth Matrix (meters) ---\n";

    for (int v = center_v - offset; v <= center_v + offset; ++v) {
        for (int u = center_u - offset; u <= center_u + offset; ++u) {
            
            // Linear index calculation
            int index = u + (v * width);

            // Access the value
            float val = depths[index];

            // Add to our string with formatting
            if (std::isnan(val)) {
                ss << "  NaN  ";
            } else {
                ss << fixed << setprecision(2) << setw(7) << val;
            }
        }
        ss << "\n"; // New line after each row
    }

    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
}

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDepthSub;
};

// The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto depth_node = std::make_shared<MinimalDepthSubscriber>();

  rclcpp::spin(depth_node);
  rclcpp::shutdown();
  return 0;
}
