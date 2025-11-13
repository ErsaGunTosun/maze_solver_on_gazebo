#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

class LidarProcessor: public rclcpp::Node
{
	public:
		LidarProcessor(): Node("lidar_processor")
		{
			subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan",
					10,std::bind(&LidarProcessor::scan_callback,this,
						std::placeholders::_1));
			RCLCPP_INFO(get_logger(),"/scan reader started");

			publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("/obstacle_data",10);

			RCLCPP_INFO(get_logger(),"/obstacle_data publisher started");
		
		}
	private:
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

		float search_area(const sensor_msgs::msg::LaserScan::SharedPtr scan, float center_angle, float fov_degrees)
        {
            float min_dist = std::numeric_limits<float>::infinity();
            
            float half_fov = (fov_degrees * M_PI / 180.0) / 2.0;
            float start_angle = center_angle - half_fov;
            float end_angle = center_angle + half_fov;

            for (int i = 0; i < (int)scan->ranges.size(); i++) {
                float current_angle = scan->angle_min + i * scan->angle_increment;

                if (current_angle >= start_angle && current_angle <= end_angle) {
                    float dist = scan->ranges[i];
                    
                    if (!std::isinf(dist) && !std::isnan(dist) && dist > scan->range_min) {
                        if (dist < min_dist) {
                            min_dist = dist;
                        }
                    }
                }
            }
            
            if (std::isinf(min_dist)) {
                return scan->range_max;
            }

            return min_dist;
        }

		void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
		{
			float front_angle = 0.0;
			float right_angle = -M_PI / 2;
			
			float front_dist = search_area(scan,front_angle,20);
			float right_dist = search_area(scan,right_angle,20);

			std_msgs::msg::Float32MultiArray msg;
			msg.data = {front_dist,right_dist};

			publisher_->publish(msg);
		}
	

};

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);

	auto node = std::make_shared<LidarProcessor>();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
