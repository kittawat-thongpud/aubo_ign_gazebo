#ifndef AUBO_ROS2_DRIVER_H
#define AUBO_ROS2_DRIVER_H

#include <array>
#include <memory>
#include <thread>
#include <mutex>
#include <queue>
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include "threadSafeQueue.h" // Assuming this is implemented elsewhere
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "aubo_ros2_common/msg/aubo_arm_event.hpp"
#include "aubo_ros2_common/msg/aubo_arm_states.hpp"
#include "aubo_ros2_common/msg/aubo_joint_states.hpp"
#include "aubo_ros2_common/msg/aubo_tcp_pose.hpp"

#include "aubo_ros2_common/srv/aubo_arm_control.hpp"
using namespace std;

namespace aubo_ros2_driver {
    constexpr int ARM_DOF = 6; // Define based on your robot's degrees of freedom
    constexpr double MAX_JOINT_ACC = 50.0/180.0*M_PI;
    constexpr double MAX_JOINT_VEL = 50.0/180.0*M_PI;
    constexpr int UPDATE_RATE_ = 10;
    constexpr int MINIMUM_BUFFER_SIZE = 300;
    const std::string joint_name_[ARM_DOF] = {"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};

    // Enumeration for robot controller mode
    enum ROBOT_CONTROLLER_MODE {
        ROBOT_CONTROLLER = 0,
        ROS_CONTROLLER
    };
    enum ControlOption
    {
        AuboAPI = 0,
        RosMoveIt
    };
    // Struct to represent a planning state
    struct PlanningState {
        double joint_pos[ARM_DOF];
        double joint_vel[ARM_DOF];
        double joint_acc[ARM_DOF];
    };

    // Struct to represent the robot state
    struct RobotState {
        aubo_robot_namespace::JointStatus joint_status_[ARM_DOF];
        aubo_robot_namespace::wayPoint_S wayPoint_;
        aubo_robot_namespace::RobotDiagnosis robot_diagnosis_info_;
        bool isRealRobotExist = false;
        bool isRobotControllerConnected = false;
        bool isMoving = false;
        ROBOT_CONTROLLER_MODE robot_controller_;
        aubo_robot_namespace::RobotState state_;
        aubo_robot_namespace::RobotErrorCode code_;
    };

    struct ArmStopped
    {
        bool user_stopped;
        bool collision_stopped;
        bool emergency_stopped;
        bool protective_stopped;
        bool singularity_stopped;
    };

    // Class to manage the robot
    class AuboRos2Driver: public rclcpp::Node {
        private:
            aubo_robot_namespace::ToolKinematicsParam tcp_;
            
            rclcpp::TimerBase::SharedPtr states_pub_timer_;
            rclcpp::Publisher<aubo_ros2_common::msg::AuboJointStates>::SharedPtr joint_states_pub_;
            rclcpp::Publisher<aubo_ros2_common::msg::AuboTcpPose>::SharedPtr tcp_pose_pub_;
            rclcpp::Publisher<aubo_ros2_common::msg::AuboArmStates>::SharedPtr arm_states_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sensor_joint_states_pub_;
            rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr fjt_feedback_pub_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr moveit_execution_pub_;

            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_control_sub_;
            rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr moveit_controller_sub_;

            rclcpp::Service<aubo_ros2_common::srv::AuboArmControl>::SharedPtr arm_control_srv_;
        public:
            // Init function
            
            AuboRos2Driver();
            // Robot state and properties
            RobotState state;
            double joint_target[ARM_DOF];
            double joint_actual[ARM_DOF];
            ThreadSafeQueue<PlanningState> joint_queue;
            std::thread *moveit_controller_thread_;

            ArmStopped arm_stopped_;
            std::string robot_ip;
            // Robot services
            ServiceInterface send_service;
            ServiceInterface receive_service;

            int rib_buffer_size_;
            int control_option_;
            // Public methods
            bool start();
            bool connectArmController();
            bool jointMove(std::vector<double> &target_joints, std::vector<double> &max_vel, std::vector<double> &max_acc);
            void handleArmStopped();
            bool checkReachTarget();
            void moveitControllerThread();

            
            void intervalStatesCallback();
            void armControlServiceCallback(const std::shared_ptr<aubo_ros2_common::srv::AuboArmControl::Request> request,
                                            std::shared_ptr<aubo_ros2_common::srv::AuboArmControl::Response> response);

            void moveitControllerCallback(const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr msg);

            void robotControlCallback(const std_msgs::msg::String::ConstSharedPtr msg);
        };

} // namespace aubo_ros2_driver

#endif // AUBO_ROS2_DRIVER_H
