#include "aubo_ros2_driver.h"

#include "serviceinterface.h"
#include "string.h"
#define SERVER_HOST "192.168.1.101"
#define SERVER_PORT 8899

using namespace aubo_ros2_driver;


AuboRos2Driver::AuboRos2Driver():Node("aubo_controller")
{
  using namespace std::placeholders;
  state.isRobotControllerConnected = false;
  state.isRealRobotExist = false;

  this->declare_parameter<std::string>("robot_ip", "192.168.1.101");
  
  this->get_parameter("robot_ip", robot_ip);
  RCLCPP_INFO(this->get_logger(), "get: robot ip: %s", robot_ip.c_str());
  tcp_.toolInEndOrientation.w = 1;
  tcp_.toolInEndOrientation.x = 0;
  tcp_.toolInEndOrientation.y = 0;
  tcp_.toolInEndOrientation.z = 0;
  tcp_.toolInEndPosition.x = 0;
  tcp_.toolInEndPosition.y = 0;
  tcp_.toolInEndPosition.z = 0;

  joint_states_pub_ = this->create_publisher<aubo_ros2_common::msg::AuboJointStates>("/aubo_robot/joint_states", 10);
  tcp_pose_pub_ = this->create_publisher<aubo_ros2_common::msg::AuboTcpPose>("/aubo_robot/tcp_pose", 10);
  arm_states_pub_ = this->create_publisher<aubo_ros2_common::msg::AuboArmStates>("/aubo_robot/arm_states", 10);

  sensor_joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  fjt_feedback_pub_ = this->create_publisher<control_msgs::action::FollowJointTrajectory_Feedback>("/aubo_robot/fjt_feedback", 10);
  moveit_execution_pub_ = this->create_publisher<std_msgs::msg::String>("/aubo_robot/moveit_execution", 10);

  moveit_controller_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    "/aubo_robot/moveit_controller", 5000, std::bind(&AuboRos2Driver::moveitControllerCallback, this, _1));

  robot_control_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/aubo_robot/robot_controller", 10, std::bind(&AuboRos2Driver::robotControlCallback, this, _1));

  arm_control_srv_ = this->create_service<aubo_ros2_common::srv::AuboArmControl>("/aubo_robot/arm_control", std::bind(&AuboRos2Driver::armControlServiceCallback, this, _1, _2));

  states_pub_timer_ = create_wall_timer(100ms, std::bind(&AuboRos2Driver::intervalStatesCallback, this));
}

bool AuboRos2Driver::start()
{
  RCLCPP_INFO(this->get_logger(), "start the driver.");

  //1.connect arm controller
  bool ret = connectArmController();

  if (!ret)
  {
    RCLCPP_INFO(this->get_logger(), "connect arm failed.");
    return false;
  }

  //2.switches to ros-controller
  int ret1 = send_service.robotServiceEnterTcp2CanbusMode();
  if (ret1 == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    RCLCPP_INFO(this->get_logger(), "Switches to ros-controller successfully");
    control_option_ = aubo_ros2_driver::RosMoveIt;
  }
  else if (ret1 == aubo_robot_namespace::ErrCode_ResponseReturnError)
  {
    ret1 = send_service.robotServiceLeaveTcp2CanbusMode();
    ret1 = send_service.robotServiceEnterTcp2CanbusMode();
    if (ret1 == aubo_robot_namespace::InterfaceCallSuccCode)
    {
      RCLCPP_INFO(this->get_logger(), "Switches to ros-controller successfully");
      control_option_ = aubo_ros2_driver::RosMoveIt;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
      control_option_ = aubo_ros2_driver::AuboAPI;
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
    control_option_ = aubo_ros2_driver::AuboAPI;
  }
  moveit_controller_thread_ = new std::thread(std::bind(&AuboRos2Driver::moveitControllerThread, this));
  moveit_controller_thread_->detach();
  RCLCPP_INFO(this->get_logger(), "aubo driver started.");
  return true;
}

bool AuboRos2Driver::connectArmController() {
    int ret1 = aubo_robot_namespace::InterfaceCallSuccCode;
    int ret2 = aubo_robot_namespace::InterfaceCallSuccCode;

    string server_host;

    if (this->get_parameter("robot_ip", server_host))
    {
        RCLCPP_INFO(this->get_logger(), "get: robot ip: %s", server_host.c_str());
    }
    else
    {
        server_host = "192.168.1.199";
        RCLCPP_INFO(this->get_logger(), "default: robot ip: %s", server_host.c_str());
    }

    //log in
  int max_link_times = 5;
  int count = 0;

  do
  {
    RCLCPP_INFO(this->get_logger(), "Connecting to arm controller...");
    count++;
    ret1 = send_service.robotServiceLogin(server_host.c_str(), 8899, "aubo", "123456");
  } while (ret1 != aubo_robot_namespace::InterfaceCallSuccCode && count < max_link_times);
  
  if (ret1 == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ret2 = receive_service.robotServiceLogin(server_host.c_str(), 8899, "aubo", "123456");
    state.isRobotControllerConnected = true;
    RCLCPP_INFO(this->get_logger(), "login success.");

    ret2 = receive_service.robotServiceGetIsRealRobotExist(state.isRealRobotExist);
    if (ret2 == aubo_robot_namespace::InterfaceCallSuccCode)
    {
      if (state.isRealRobotExist)
        RCLCPP_INFO(this->get_logger(), "real robot exist.");
      else
        RCLCPP_INFO(this->get_logger(), "real robot does not exist.");
    }
  }
  else
  {
    state.isRobotControllerConnected = false;
    RCLCPP_INFO(this->get_logger(), "login failed.");
  state.isRobotControllerConnected = false; 
    return false;
  }
  
  state.isRobotControllerConnected = true;
  return true;
  
}

bool AuboRos2Driver::jointMove(std::vector<double> &target_joints, std::vector<double> &max_vel, std::vector<double> &max_acc)
{
  int ret = aubo_robot_namespace::InterfaceCallSuccCode;
  bool result = false;

  ret = send_service.robotServiceLeaveTcp2CanbusMode();
  if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    control_option_ = aubo_ros2_driver::AuboAPI;
  }
  else
    return false;
  
  ret = send_service.robotServiceInitGlobalMoveProfile();

  aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
  aubo_robot_namespace::JointVelcAccParam jointMaxVelc;

  for (int i = 0; i < 6; i++)
  {
    jointMaxAcc.jointPara[i] = max_acc[i];
    jointMaxVelc.jointPara[i] = max_vel[i];
  }

  ret = send_service.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
  ret = send_service.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

  double joints[6];
  for (int i = 0; i < 6; i++)
  {
    joints[i] = target_joints[i];
  }

  ret = send_service.robotServiceJointMove(joints, false);

  if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    result = true;
    RCLCPP_INFO(this->get_logger(), "joint move success.");
  }
  else
  {
    result = false;
    RCLCPP_INFO(this->get_logger(), "joint move failed. errCode: %d", ret);
  }

  ret = send_service.robotServiceEnterTcp2CanbusMode();
  if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    control_option_ = aubo_ros2_driver::RosMoveIt;
  }
  
  return result;
}

bool AuboRos2Driver::checkReachTarget()
{  
  bool ret = true;
  if (state.robot_controller_ == ROBOT_CONTROLLER)
    for (int i = 0; i < ARM_DOF; i++)
    {
      if(fabs(joint_target[i] - joint_actual[i]) > 0.001)
      {
        ret = false;
      }
    }
  else
    {
      if (joint_queue.isEmpty()) {
        ret = true;
      }
      else
      {
        ret = false;
      }
    }
  return ret;

}

void AuboRos2Driver::moveitControllerThread()
{
  rclcpp::Rate loop_rate(UPDATE_RATE_);

  while (rclcpp::ok())
  {
    if(joint_queue.size() > 0 && !(state.isMoving))
    {
      state.isMoving = true;
    }
    if (state.isMoving && rib_buffer_size_ < MINIMUM_BUFFER_SIZE)
    {
      if (joint_queue.size() > 0)
      {
        PlanningState ps;
        joint_queue.pop(ps);
        send_service.robotServiceSetRobotPosData2Canbus(ps.joint_pos);
      }
      else
        state.isMoving = false;
    }

    loop_rate.sleep();
  }
  
}

void AuboRos2Driver::handleArmStopped()
{
  send_service.robotMoveFastStop();
  if(joint_queue.size() > 0)
  {
    std_msgs::msg::String msg;
    msg.data = "stop";
    moveit_execution_pub_->publish(msg);
  }

  while (joint_queue.size() > 0)
  {
    joint_queue.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "handle arm stopped");
}

void AuboRos2Driver::robotControlCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "receive cmd: %s", msg->data.c_str());
  int ret = aubo_robot_namespace::InterfaceCallSuccCode;

  if (msg->data == "powerOn")
  {
    int collision_level = 7;
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    toolDynamicsParam.positionX = 0.0;
    toolDynamicsParam.positionY = 0.0;
    toolDynamicsParam.positionZ = 0.0;
    toolDynamicsParam.payload = 1.0;

    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    ret = receive_service.rootServiceRobotStartup(toolDynamicsParam /**工具动力学参数**/,
                                                      collision_level /*碰撞等级*/,
                                                      true /*是否允许读取位姿　默认为true*/,
                                                      true,    /*保留默认为true */
                                                      1000,    /*保留默认为1000 */
                                                      result); /*机械臂初始化*/

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "powerOn success.");
    else
      RCLCPP_INFO(this->get_logger(), "powerOn failed.");
  }
  else if (msg->data == "powerOff")
  {
    ret = receive_service.robotServiceRobotShutdown(true);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "powerOff success.");
    else
      RCLCPP_INFO(this->get_logger(), "powerOff failed.");
  }
}

void AuboRos2Driver::armControlServiceCallback(const std::shared_ptr<aubo_ros2_common::srv::AuboArmControl::Request> request,
                                               std::shared_ptr<aubo_ros2_common::srv::AuboArmControl::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "request cmd: %s", request->cmd.c_str());

  int ret = aubo_robot_namespace::InterfaceCallSuccCode;

  if (request->cmd == "powerOn")
  {
    int collision_level = 9;
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    toolDynamicsParam.positionX = 0.0;
    toolDynamicsParam.positionY = 0.0;
    toolDynamicsParam.positionZ = 0.0;
    toolDynamicsParam.payload = 1.0;

    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    ret = receive_service.rootServiceRobotStartup(toolDynamicsParam /**工具动力学参数**/,
                                                      collision_level /*碰撞等级*/,
                                                      true /*是否允许读取位姿　默认为true*/,
                                                      true,    /*保留默认为true */
                                                      1000,    /*保留默认为1000 */
                                                      result); /*机械臂初始化*/

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "powerOn success.");
    else
      RCLCPP_INFO(this->get_logger(), "powerOn failed.");
  }
  else if (request->cmd == "powerOff")
  {
    ret = receive_service.robotServiceRobotShutdown(true);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "powerOff success.");
    else
      RCLCPP_INFO(this->get_logger(), "powerOff failed.");
  }
  else if (request->cmd == "stop")
  {
    handleArmStopped();
  }
  else if (request->cmd == "collision recover")
  {
    ret = receive_service.robotServiceCollisionRecover();

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "collision recover success.");
    else
      RCLCPP_INFO(this->get_logger(), "collision recover failed.");
  }
  else if (request->cmd == "clear singularityOverSpeedAlarm")
  {
    ret = receive_service.rootServiceRobotControl(aubo_robot_namespace::RobotControlCommand::ClearSingularityOverSpeedAlarm);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "clear singularityOverSpeedAlarm success.");
    else
      RCLCPP_INFO(this->get_logger(), "clear singularityOverSpeedAlarm failed.");
  }
  else if (request->cmd == "setTcp")
  {
    tcp_.toolInEndPosition.x = request->tcp.position.x;
    tcp_.toolInEndPosition.y = request->tcp.position.y;
    tcp_.toolInEndPosition.z = request->tcp.position.z;
    tcp_.toolInEndOrientation.w = request->tcp.orientation.w;
    tcp_.toolInEndOrientation.x = request->tcp.orientation.x;
    tcp_.toolInEndOrientation.y = request->tcp.orientation.y;
    tcp_.toolInEndOrientation.z = request->tcp.orientation.z;

    ret = receive_service.robotServiceSetRobotTool(tcp_);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "set robot tool success.");
    else
      RCLCPP_INFO(this->get_logger(), "set robot tool failed.");

    ret = receive_service.robotServiceSetToolKinematicsParam(tcp_);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "set tcp kinematics success.");
    else
      RCLCPP_INFO(this->get_logger(), "set tcp kinematics failed.");
  }
  else if (request->cmd == "setPayload")
  {
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    toolDynamicsParam.positionX = 0.0;
    toolDynamicsParam.positionY = 0.0;
    toolDynamicsParam.positionZ = 0.0;
    toolDynamicsParam.payload = request->payload;
    ret = receive_service.robotServiceSetToolDynamicsParam(toolDynamicsParam);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "set payload success.");
    else
      RCLCPP_INFO(this->get_logger(), "set payload failed.");
  }
  else if (request->cmd == "jointMove")
  {
    state.isMoving = true;

    RCLCPP_INFO(this->get_logger(), "set payload failed.");
    std::vector<double> maxAcc, maxVel;
    if ( request->velocities.size() != 6 || request->accelerations.size() != 6)
    {
      for (int i = 0; i < 6; i++)
      {
        maxAcc.push_back(MAX_JOINT_ACC);
        maxVel.push_back(MAX_JOINT_VEL);
      }
    }
    else 
    {
      for (int i = 0; i < 6; i++)
      {
        maxAcc.push_back(request->accelerations[i]);
        maxVel.push_back(request->velocities[i]);
      }
    }

    RCLCPP_INFO(this->get_logger(), "set payload failed.");
    bool result = jointMove(request->joints, maxVel, maxAcc);

    state.isMoving = false;

    if (result)
      ret = aubo_robot_namespace::InterfaceCallSuccCode;
    else
      ret = aubo_robot_namespace::ErrCode_Failed;
  }
  else if (request->cmd == "setCollisionLevel")
  {
    ret = receive_service.robotServiceSetRobotCollisionClass(request->collision_level);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "set collision level %d success.", request->collision_level);
    else
      RCLCPP_INFO(this->get_logger(), "set collision level failed.");
  }
  else
  {
    response->result = false;
    response->result_code = "No Command";
    return;
  }

  if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    response->result = true;
    response->result_code = ret;
  }
  else
  {
    response->result = false;
    response->result_code = ret;
  }
}

void AuboRos2Driver::moveitControllerCallback(const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr msg)
{
  PlanningState ps;
  for(int i = 0; i < 6; i++)
  {
    ps.joint_pos[i] = msg->positions[i];
    ps.joint_vel[i] = msg->velocities[i];
    ps.joint_acc[i] = msg->accelerations[i];
  }
  
  //RCLCPP_INFO(this->get_logger(), "Driver push ps %f|%f|%f|%f|%f|%f|",ps.joint_pos[0],ps.joint_pos[1],ps.joint_pos[2],ps.joint_pos[3],ps.joint_pos[4],ps.joint_pos[5]);
  joint_queue.push(ps);
}

void AuboRos2Driver::intervalStatesCallback()
{
  if (state.isRobotControllerConnected && state.isRealRobotExist)
  {
    int ret = receive_service.robotServiceGetCurrentWaypointInfo(state.wayPoint_);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
      for (int i = 0; i < 6; i++)
        joint_actual[i] = state.wayPoint_.jointpos[i];
    }

    // get joint states
    receive_service.robotServiceGetRobotJointStatus(state.joint_status_, 6);
    aubo_ros2_common::msg::AuboJointStates joint_states;
    for (int i = 0; i < 6; i++)
    {
      joint_states.actual_current.push_back(state.joint_status_[i].jointCurrentI);
      joint_states.target_current.push_back(state.joint_status_[i].jointTagCurrentI);
      joint_states.actual_position.push_back(state.joint_status_[i].jointPosJ);
      joint_states.target_position.push_back(state.joint_status_[i].jointTagPosJ);
    }
    joint_states_pub_->publish(joint_states);

    // get tcp pose
    aubo_robot_namespace::Pos actual_toolEndPositionOnBase;
    aubo_robot_namespace::Ori actual_toolEndOrientationOnBase;
    receive_service.baseToBaseAdditionalTool(state.wayPoint_.cartPos.position, state.wayPoint_.orientation, tcp_, actual_toolEndPositionOnBase, actual_toolEndOrientationOnBase);
    aubo_ros2_common::msg::AuboTcpPose tcp_pose;
    tcp_pose.actual_pose.position.x = actual_toolEndPositionOnBase.x;
    tcp_pose.actual_pose.position.y = actual_toolEndPositionOnBase.y;
    tcp_pose.actual_pose.position.z = actual_toolEndPositionOnBase.z;
    tcp_pose.actual_pose.orientation.w = actual_toolEndOrientationOnBase.w;
    tcp_pose.actual_pose.orientation.x = actual_toolEndOrientationOnBase.x;
    tcp_pose.actual_pose.orientation.y = actual_toolEndOrientationOnBase.y;
    tcp_pose.actual_pose.orientation.z = actual_toolEndOrientationOnBase.z;
    tcp_pose_pub_->publish(tcp_pose);

    // get arm states
    aubo_ros2_common::msg::AuboArmStates arm_states;

    receive_service.robotServiceGetRobotDiagnosisInfo(state.robot_diagnosis_info_);
    rib_buffer_size_ = state.robot_diagnosis_info_.macTargetPosDataSize;
    arm_states.control_option = control_option_;
    int collision_level = 0;
    receive_service.robotServiceGetRobotCollisionCurrentService(collision_level);
    arm_states.collision_level = collision_level;
    arm_states.arm_power_status = state.robot_diagnosis_info_.armPowerStatus;
    arm_states.collision_stopped = state.robot_diagnosis_info_.robotCollision;
    arm_states.emergency_stopped = state.robot_diagnosis_info_.softEmergency;
    arm_states.singularity_stopped = state.robot_diagnosis_info_.singularityOverSpeedAlarm;
    arm_states.protective_stopped = false;
    arm_states.reached_target = true;
    
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;

    receive_service.robotServiceGetToolDynamicsParam(toolDynamicsParam);
    arm_states.payload = toolDynamicsParam.payload;

    bool is_timeout;
    receive_service.robotServiceGetConnectStatus(is_timeout);
    arm_states.is_timeout = is_timeout;
    arm_states.in_motion = state.isMoving;

    arm_states_pub_->publish(arm_states);

    if (arm_states.emergency_stopped || arm_states.collision_stopped || arm_states.protective_stopped || arm_states.singularity_stopped)
    {
      handleArmStopped();
      RCLCPP_INFO(this->get_logger(), "|%d|%d|%d|%d|",arm_states.emergency_stopped,arm_states.collision_stopped,arm_states.protective_stopped,arm_states.singularity_stopped);
    }

    //pub sensor joint states
    sensor_msgs::msg::JointState sensor_joint_states;
    sensor_joint_states.header.stamp = this->now();
    sensor_joint_states.name.resize(ARM_DOF);
    sensor_joint_states.position.resize(ARM_DOF);
    for (int i = 0; i < 6; i++)
    {
      sensor_joint_states.name[i] = joint_name_[i];
      sensor_joint_states.position[i] = state.joint_status_[i].jointPosJ;
    }
    sensor_joint_states_pub_->publish(sensor_joint_states);

    //pub fjt joint feedback
    control_msgs::action::FollowJointTrajectory_Feedback joint_feedback;
    joint_feedback.header.stamp = this->now();
    for (int i = 0; i < 6; i++)
    {
      joint_feedback.joint_names.push_back(joint_name_[i]);
      joint_feedback.actual.positions.push_back(state.joint_status_[i].jointPosJ);
    }
    fjt_feedback_pub_->publish(joint_feedback);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "waiting controller and arm available.");
  }
}