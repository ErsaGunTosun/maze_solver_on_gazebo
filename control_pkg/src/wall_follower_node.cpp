#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"


class WallFollower: public rclcpp::Node
{
    public:
        WallFollower(): Node("wall_follower")
		{
            declare_parameter("target_dist", 0.35);      
            declare_parameter("min_front_dist", 0.5);   
            declare_parameter("fwd_speed", 0.1);       
            declare_parameter("turn_speed", 0.25);       

            target_dist_ = get_parameter("target_dist").as_double();
            min_front_dist_ = get_parameter("min_front_dist").as_double();
            fwd_speed_ = get_parameter("fwd_speed").as_double();
            turn_speed_ = get_parameter("turn_speed").as_double();

			subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>("/obstacle_data",
					10,std::bind(&WallFollower::obstacle_callback,this,
						std::placeholders::_1));
			RCLCPP_INFO(get_logger(),"/scan reader started");	
            
            publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
		}

    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

        double target_dist_;
        double min_front_dist_;
        double fwd_speed_;
        double turn_speed_;

        void obstacle_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            if (msg->data.size() < 2) {
                RCLCPP_WARN(get_logger(), "Gelen Lidar verisi eksik!");
                return;
            }

            float front_dist = msg->data[0];
            float right_dist = msg->data[1];

            // Gönderilecek hareket komutu mesajını hazırla
            auto twist_msg = geometry_msgs::msg::Twist();

            // ---- SAĞ DUVAR TAKİBİ MANTIĞI (Projenin Beyni) ----
            // Bu bir öncelik sırasıdır. En üstteki kural en önemlidir.

            // 1. KURAL (ACİL DURUM): Önümüzde engel var mı?
            if (front_dist < min_front_dist_)
            {
                // Çarpmamak için dur ve SOLA dön (sağdaki duvardan uzağa)
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = turn_speed_;
                RCLCPP_INFO(get_logger(), "KURAL 1: ON ENGEL! Sola don.");
            }
            // 2. KURAL (KÖŞE DÖNME): Sağımızda duvar yok mu (boşluk)?
            else if (right_dist > target_dist_ * 2.0) // İdeal mesafenin 2 katından fazlaysa
            {
            // Köşeyi dönmek için yavaşça ilerle ve SAĞA dön (yeni duvarı bul)
            twist_msg.linear.x = fwd_speed_ * 0.5; // Dönerken yavaşla
            twist_msg.angular.z = -turn_speed_; // Negatif = Sağa Dönüş
            RCLCPP_INFO(get_logger(), "KURAL 2: KOSE! Saga don.");
            }
            // 3. KURAL (DUVARA YAKIN): Duvara çok mu yakınız?
            else if (right_dist < target_dist_ - 0.1) // İdeal mesafeden 10cm daha yakın
            {
                // Duvardan uzaklaşmak için ilerle ve hafif SOLA dön
                twist_msg.linear.x = 0;
                twist_msg.angular.z = turn_speed_ * 0.5; // Hafif dönüş
                RCLCPP_INFO(get_logger(), "KURAL 3: Cok YAKIN. Sola ac.");
            }
            // 4. KURAL (DUVARA UZAK): Duvardan çok mu uzağız?
            else if (right_dist > target_dist_ + 0.1) // İdeal mesafeden 10cm daha uzak
            {
                // Duvara yaklaşmak için ilerle ve hafif SAĞA dön
                twist_msg.linear.x = fwd_speed_;
                twist_msg.angular.z = -turn_speed_ * 0.5; // Hafif dönüş
                RCLCPP_INFO(get_logger(), "KURAL 4: Cok UZAK. Saga yaklas.");
            }
            // 5. KURAL (İDEAL DURUM): Tam istediğimiz gibi.
            else
            {
                // Dümdüz ilerle
                twist_msg.linear.x = fwd_speed_;
                twist_msg.angular.z = 0.0;
                RCLCPP_INFO(get_logger(), "KURAL 5: IDEAL. Duz git.");
            }

            // Karar verilen hareketi /cmd_vel'e yayınla
            publisher_->publish(twist_msg);
        }
};

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);

	auto node = std::make_shared<WallFollower>();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}