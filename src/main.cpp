#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

class PointCloudORL : public rclcpp::Node
{
public:
  PointCloudORL() : Node("ORLcloud")
  {
    // Qos settings
    rclcpp::QoS qos(10);
    qos.best_effort();

    // create subscription from zed
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed/zed_node/point_cloud/cloud_registered",
      qos, // in terminal command specify draco compression subscription
      std::bind(&PointCloudORL::cloudCallback, this, std::placeholders::_1)
    );

    // publish the math cloud
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ORLcloud", 10);
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to: /zed/zed_node/point_cloud/cloud_registered");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /ORLcloud");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB> input_cloud; //convert raw cloud to PCL cloud
    pcl::fromROSMsg(*msg, input_cloud);

    if (input_cloud.empty()) 
      return;

    pcl::PointCloud<pcl::PointXYZRGB> display_cloud;
    
    float z_ref = 1000.0f;
    float z1 = -1000.0f;  
    int valid_points = 0;

    const float MIN_DIST = 0.1f;  // 1 meter away
    const float MAX_DIST = 2.5f;  // 2.5 meters away
    const float WIDTH = 0.5f;  

    for (const auto& pt : input_cloud.points)
    {
        if (!std::isfinite(pt.z)) 
            continue;

            // setting FoV
        if (pt.x > MIN_DIST && pt.x < MAX_DIST && std::abs(pt.y) < WIDTH) 
        {
            display_cloud.push_back(pt);

            if (pt.z < z_ref) 
              z_ref = pt.z; 
            if (pt.z > z1) 
              z1 = pt.z;

            valid_points++;
        }
    }

    if (!display_cloud.empty()) {
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(display_cloud, output_msg);
        
        output_msg.header = msg->header;
        pub_->publish(output_msg);
    }

    if (valid_points > 10) 
    {
        float height = z1 - z_ref;
        if (height < 0.02f) 
          height = 0.0f;
        std::cout << "\rPoints: " << valid_points 
         << " | Floor: " << std::fixed << std::setprecision(3) << z_ref 
         << " | Height: " << height << "m   " << std::flush;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudORL>());
  rclcpp::shutdown();
  return 0;
}