#ifndef HAPTIC_DEVICE_H__
#define HAPTIC_DEVICE_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <boost/thread/thread.hpp>
#include <mutex>
#include <vector>
#include "dhdc.h"
#include "std_msgs/msg/int8_multi_array.hpp"



class HapticDevice
{
	public:
        HapticDevice(std::shared_ptr<rclcpp::Node> node, float loop_rate, bool set_force);
        virtual ~HapticDevice();

        void PublishHapticData();
        void RegisterCallback();

        void GetHapticDataRun();
		
        void SetForce(double x, double y, double z);

        void SetForceLimit(double x, double y, double z);
        void VerifyForceLimit(double input_force[], std::vector<double> & output);
        void ForceCallback(const geometry_msgs::msg::Vector3 &data);

        void Start();

    protected:
       std::shared_ptr<boost::thread> dev_op_thread_;

	private:
       std::shared_ptr<rclcpp::Node> nh_;
       rclcpp::Rate loop_rate_;
       int device_count_;
       int dev_id_;
       bool set_force_;
       bool keep_alive_=false;
       bool device_enabled_ = false;
       std::mutex val_lock_;
       bool force_released_;
       bool button0_state_=false;
       bool button1_state_=false;

       std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>> position_pub_;
       std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int8MultiArray>> button_state_pub_;
       std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Vector3>> force_sub_;
       double position_[3];

       std::string position_topic_;
       std::string buttons_topic_;
       std::string force_topic_;

       double force_x_limit_;
       double force_y_limit_;
       double force_z_limit_;
       std::vector<double> force_;

};

#endif
