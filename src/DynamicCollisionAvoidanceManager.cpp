/*
 * Copyright (C) 2024 IIT-HHCM
 * Author: Liana Bertoni
 * email:  liana.bertoni@iit.it
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <DynamicCollisionAvoidance/DynamicCollisionAvoidanceManager.h>

DynamicCollisionAvoidanceManager::DynamicCollisionAvoidanceManager ( std::string ns )
{
    // ros2 'init'
    _nh = rclcpp::Node::make_shared(ns);

    //initialization ROS node
    RCLCPP_INFO(_nh->get_logger(), "%s\n","I am initializing the ROS node...");
    initROSNode();

    //initialization ROS node
    RCLCPP_INFO(_nh->get_logger(), "%s\n","I am loading the ROS node params...");
    loadParam();
}

void DynamicCollisionAvoidanceManager::initROSNode()
{
    // init ROS node
    int rate;
    _nh->declare_parameter("rate", 1000);
    _nh->get_parameter("rate", rate);
    _timer = _nh->create_wall_timer( std::chrono::milliseconds(rate),std::bind(&DynamicCollisionAvoidanceManager::timer_callback, this));

    // ros topic param
    _nh->declare_parameter("topic_motion_subscriber_name","motion_planner");
    _nh->declare_parameter("topic_sensors_subscriber_name","sensors_data");
    _nh->declare_parameter("topic_robot_publisher_name","robot_local_planner");
    _nh->get_parameter("topic_motion_subscriber_name",_topicMotionSub);
    _nh->get_parameter("topic_sensors_subscriber_name",_topicSensorsSub);
    _nh->get_parameter("topic_robot_publisher_name",_topicRobotPub);
    
    _motionSubscriber = _nh->create_subscription<dyn_collision_avoid::msg::MotionMsg>(_topicMotionSub,1, std::bind(&DynamicCollisionAvoidanceManager::motionSubscriberCallback, this, _1));
    _sensorsSubscriber = _nh->create_subscription<dyn_collision_avoid::msg::SensorsMsg>(_topicSensorsSub,1, std::bind(&DynamicCollisionAvoidanceManager::sensorsSubscriberCallback, this, _1));
    _robotPublisher = _nh->create_publisher<dyn_collision_avoid::msg::RobotMsg>(_topicRobotPub,1000);
}

void DynamicCollisionAvoidanceManager::loadParam()
{
    std::vector<double> q0;
    std::vector<std::string> listFrames;

    // declaring params with default values
    _nh->declare_parameter("robot_urdf_model_path","/tmp/robot.urdf");
    _nh->declare_parameter("robot_base_frame_name","base_link");
    _nh->declare_parameter("robot_tip_frame_name","end_effector");
    _nh->declare_parameter("sensors_frame_name",listFrames);
    _nh->declare_parameter("robot_initial_config",q0);
    _nh->declare_parameter("correction_time",0.8);
    _nh->declare_parameter("distance_threshold",0.20);
    _nh->declare_parameter("n_sensors",6);

    // getting params with their proper values
    _nh->get_parameter("robot_initial_config",q0);
    _nh->get_parameter("robot_urdf_model_path",_robotURDFModelPath);
    _nh->get_parameter("robot_base_frame_name",_base);
    _nh->get_parameter("robot_tip_frame_name",_endEffector);
    _nh->get_parameter("sensors_frame_name",_sFrameName);

    for (int i = 0; i < _sFrameName.size(); ++i)
        std::cout << "frame names : " << _sFrameName[i] << std::endl;

    _nh->get_parameter("correction_time",_ref_time);
    _nh->get_parameter("distance_threshold",_threshold);
    _nh->get_parameter("n_sensors",_ns);

    // init robot model
    _kdl.init(_robotURDFModelPath,_base,_endEffector);
    _kdl.setJointsPositions(_utils.toEigen(q0));
    _kdl.FK();
    _target_pose_i = _kdl.getPose();
    _target_pose = _target_pose_i;

    // print initial status
    std::cout << "q0 : " << _utils.toEigen(q0).transpose() << std::endl;
    std::cout << "p0 : " << _target_pose.transpose() << std::endl;

    // cartesian space
    _target_pose_delta = Eigen::VectorXd::Zero(_nc);
    _target_pose_delta_i = Eigen::VectorXd::Zero(_nc);
    _target_pose_delta_prev = Eigen::VectorXd::Zero(_nc);
    _target_pose_i_received = Eigen::VectorXd::Zero(_nc);

    // joint space
    _nj = q0.size();
    _qref = Eigen::VectorXd::Zero(_nj);
    _sensors = Eigen::VectorXd::Zero(_ns);

    // sesnsors initial settings
    _distances = 2.0 * Eigen::VectorXd::Ones(_ns);
    _sFrame = Eigen::VectorXd::Zero(3);
    _sFrame(2) = -1; // sensor axis

    _dt = 0.001; // loop time
    _it_time = 0.0;

    // state
    _subscribedData = false;
    _planCorrection = false;
    _collisionFound = false;
    _subscribedTarget = false;

    // logger init
    _nh->declare_parameter("log_path","/tmp/");
    _nh->get_parameter("log_path",_logPath);
    _loggerL.initLogger(_logPath);
    _loggerL.setLogger(false); // false:unlimited true:limited buffer
}

void DynamicCollisionAvoidanceManager::checkCollision()
{
    // _collisionFound = false;
    _target_pose_delta = Eigen::VectorXd::Zero(_nc);
    _sensors = Eigen::VectorXd::Zero(_ns);

    for (int i = 0; i < _ns; ++i)
    {   
        // std::cout << "is it violating the threshold? " << i << std::endl;
        if(_distances[i] < _threshold)
        {
            _sensors[i] = 1;
            // _collisionFound = true;
            _planCorrection = true;
            _target_pose_delta = computeVariation();    
            // std::cout << i << " yes" << std::endl;
        }
    }   
}

Eigen::VectorXd DynamicCollisionAvoidanceManager::computeVariation()
{
    Eigen::VectorXd target_delta;
    target_delta = Eigen::VectorXd::Zero(_nc);

    // std::cout << "Sensor violation compute variation " << std::endl;
    for (int i = 0; i < _ns; ++i)
    {
        if(_sensors[i])
        {
            // std::cout << "Sensor violation n: " << i+1 << " frame name: " << _sFrameName[i] << std::endl;
            Eigen::MatrixXd T_s;
            Eigen::MatrixXd R_s;
            Eigen::VectorXd p_s;

            T_s = _kdl.getTransform(_base,_sFrameName[i]);
            // std::cout << "T_s " << T_s << std::endl;

            R_s = T_s.block(0,0,3,3); // get rot
            // std::cout << "R_s " << T_s << std::endl;

            p_s = ( _threshold - _distances[i]) * _sFrame; // -1 inside the sFrame mapping
            // std::cout << "p_s " << T_s << std::endl;

            target_delta.head(3) = target_delta.head(3) + R_s * p_s;
        }
    }

    std::cout << "variation is " << target_delta.transpose() << std::endl;

    return target_delta;
}

void DynamicCollisionAvoidanceManager::planCorrection()
{
    double t = _it_time/_ref_time;
    double s = ((6*t - 15)*t + 10)*t*t*t;

    if(t < 1.0)   
        _target_pose_delta_i = _target_pose_delta_prev + s * ( _target_pose_delta - _target_pose_delta_prev);
    else
    {
        _planCorrection = false;
        _it_time = 0.0;
    }

    // next step
    _it_time += _dt;
}

void DynamicCollisionAvoidanceManager::timer_callback()
{
    // control loop 
    if (_subscribedData)
    {
        // check coming collision
        // std::cout << "check collision..."<< std::endl;
        checkCollision();
        _subscribedData = false;
    }

    if(_planCorrection)
    {
        planCorrection();
    }

    if(_subscribedTarget)
    {
        _target_pose_i = _target_pose_i_received;
        _subscribedTarget = false;
    }

    // targe pose
    _target_pose = _target_pose_i + _target_pose_delta_i;
    _qref = IK(_target_pose);

    composeRobotMsg();
    _robotPublisher->publish(_robotMsg);

    // log
    log();
}

void DynamicCollisionAvoidanceManager::composeRobotMsg()
{
    // compose message
    _robotMsg.target_pose = _utils.toStdVector(_target_pose);
    _robotMsg.joints_position_reference = _utils.toStdVector(_qref);
}

void DynamicCollisionAvoidanceManager::sensorsSubscriberCallback(const dyn_collision_avoid::msg::SensorsMsg::SharedPtr msg)
{
    // sub sensors data
    _distances = _utils.toEigen(msg->distances);
    // std::cout << "dist_meas : " << _distances.transpose() << std::endl;

    // for the iteration
    // _it_time = 0.0;
    _target_pose_delta_prev = _target_pose_delta_i;
    _subscribedData = true;
}

void DynamicCollisionAvoidanceManager::motionSubscriberCallback(const dyn_collision_avoid::msg::MotionMsg::SharedPtr msg)
{
    _target_pose_i_received = _utils.toEigen(msg->task_pose_reference);
    _subscribedTarget = true;
}

Eigen::VectorXd DynamicCollisionAvoidanceManager::IK(Eigen::VectorXd t_ref)
{
    // FK
    _kdl.setPose(t_ref);
    _kdl.IK();

    return _kdl.getJointsPositions();
}

void DynamicCollisionAvoidanceManager::spin()
{
    rclcpp::spin(_nh);
}

void DynamicCollisionAvoidanceManager::log()
{
    // log data
    _loggerL.logData("TargetPoseReceived",_target_pose_i);
    _loggerL.logData("TargetPoseCorrectionTerm",_target_pose_delta_i);
    _loggerL.logData("TargetPoseFinal",_target_pose);
    _loggerL.logData("JointsPositionReference",_qref);
}

DynamicCollisionAvoidanceManager::~DynamicCollisionAvoidanceManager()
{

}