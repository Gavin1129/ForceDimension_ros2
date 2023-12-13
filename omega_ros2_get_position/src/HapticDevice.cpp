#include "omega_ros2_get_position/HapticDevice.h"
// haptic device API
#include "omega_ros2_get_position/dhdc.h"
#include "std_msgs/msg/int8_multi_array.hpp"


HapticDevice::HapticDevice(std::shared_ptr<rclcpp::Node> node, float loop_rate, bool set_force): loop_rate_(loop_rate)
{
    nh_ = node;

    dev_id_ = -2; // we set a value not in API defined range
    device_enabled_ = -1;

    set_force_ = set_force;

    for (int i = 0; i<3;i++) {
        position_[i] = 0.0;
    }

    button0_state_ = false;
    keep_alive_ = false;
    force_released_ = true;

    force_.resize(3);
    force_[0] = 0.0;
    force_[1] = 0.0;
    force_[2] = 0.0;

    SetForceLimit(10.0, 10.0, 10.0);

    // connect to hardware device
    device_count_ = dhdGetDeviceCount();

    // we only accept one haptic device.
    if ( device_count_ >= 1) {
        dev_id_ = dhdOpenID(0); // if open failed, we will get -1, and sucess with 0.
        if ( dev_id_ < 0) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"error: handler device: %s\n", dhdErrorGetLastStr());
            device_enabled_ = false;
            return;
        }
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"No handler device find! %s\n", dhdErrorGetLastStr());
        device_enabled_ = false;
        return;
    }

    device_enabled_ =true;
}


HapticDevice::~HapticDevice()
{
    dev_id_ = -1;
    device_count_ = 0;
    keep_alive_ = false;
    if (dev_op_thread_)
    {
        dev_op_thread_->join();
    }
}

void HapticDevice::PublishHapticData()
{
    geometry_msgs::msg::Vector3Stamped pos;
    pos.header.frame_id = nh_->get_name();
    pos.header.stamp = nh_->get_clock()->now();
    pos.vector.x = position_[0];
    pos.vector.y = position_[1];
    pos.vector.z = position_[2];

    std_msgs::msg::Int8MultiArray button_stat;
    button_stat.data.push_back(button0_state_);
    button_stat.data.push_back(button1_state_);

    position_pub_->publish(pos);
    button_state_pub_->publish(button_stat);
}

void HapticDevice::RegisterCallback()
{
    position_topic_ = "/haptic/position";
    buttons_topic_ = "/haptic/button_state";
    force_topic_ = "/haptic/force";

    position_pub_ = nh_->create_publisher<geometry_msgs::msg::Vector3Stamped>(position_topic_.c_str(),1);
    button_state_pub_ = nh_->create_publisher<std_msgs::msg::Int8MultiArray>(buttons_topic_.c_str(), 1);
    force_sub_ = nh_->create_subscription<geometry_msgs::msg::Vector3>(force_topic_.c_str(), 1, 
                                            std::bind(&HapticDevice::ForceCallback, this, std::placeholders::_1)
);

}       

void HapticDevice::ForceCallback(const geometry_msgs::msg::Vector3 &data)
{
    // wrapper force
    SetForce(data.x, data.y, data.z);
}

void HapticDevice::GetHapticDataRun()
{   // get and we will publish immediately


    double feed_force[3] = {0.0, 0.0, 0.0};
    double current_position[3] = {0.0, 0.0, 0.0};

    while (rclcpp::ok() && (keep_alive_ == true)) {

        if (device_count_ >= 1 && dev_id_ >= 0) {
                dhdGetPosition(&current_position[0], &current_position[1], &current_position[2]);
                position_[0] = current_position[0];
                position_[1] = current_position[1];
                position_[2] = current_position[2];
                RCLCPP_INFO(nh_->get_logger(),"position is: %f ,%f ,%f", position_[0], position_[1], position_[2]);
                button0_state_ = dhdGetButton(0, dev_id_);
        }

        PublishHapticData();

        // apply force
        if (set_force_) {
            val_lock_.lock();
            feed_force[0] = force_[0];
            feed_force[1] = force_[1];
            feed_force[2] = force_[2];
            dhdSetForce(feed_force[0], feed_force[1], feed_force[2]);
            val_lock_.unlock();
        }


        loop_rate_.sleep();
    }

}

void HapticDevice::SetForce(double x, double y, double z)
{
    double input_force[3] = {0.0, 0.0, 0.0};

    if (set_force_)
    {
        val_lock_.lock();
        input_force[0] = x;
        input_force[1] = y;
        input_force[2] = z;
        VerifyForceLimit(input_force, force_);
        force_released_ = false;
        val_lock_.unlock();
    }
}

void HapticDevice::SetForceLimit(double x, double y, double z)
{
    force_x_limit_ = x;
    force_y_limit_ = y;
    force_z_limit_ = z;
}


void HapticDevice::VerifyForceLimit(double input_force[], std::vector<double> & output)
{
    if (output.size() != 3) {
        output.resize(3);
    }

    if (input_force[0] < -force_x_limit_) output[0] = -force_x_limit_;
    if (input_force[1] < -force_y_limit_) output[1] = -force_y_limit_;
    if (input_force[2] < -force_z_limit_) output[2] = -force_z_limit_;

    if (input_force[0] > force_x_limit_) output[0] = force_x_limit_;
    if (input_force[1] > force_y_limit_) output[1] = force_y_limit_;
    if (input_force[2] > force_z_limit_) output[2] = force_z_limit_;
}


void HapticDevice::Start()
{   
    if (!device_enabled_)
    {
        return; 
    }

    RegisterCallback();

    dev_op_thread_ = std::make_shared<boost::thread>(&HapticDevice::GetHapticDataRun, this);
    keep_alive_ = true;

    // Create a MultiThreadedExecutor for handling callbacks in separate threads
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh_); // assuming nh_ is your shared pointer to a Node

    // Run the executor in a separate thread
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    // Main loop
    while (rclcpp::ok() && keep_alive_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // ROS_INFO("working in main loop");
        // usleep(1000);
    }

    keep_alive_ = false;

    // Stop the executor and join the thread
    executor.cancel();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
}
