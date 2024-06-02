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

#ifndef DYNAMIC_COLLISION_AVOIDANCE_
#define DYNAMIC_COLLISION_AVOIDANCE_

// ROS 
#include <sstream>
#include "rclcpp/rclcpp.hpp"

// Custom Messages    
#include "dyn_collision_avoid/msg/motion_msg.hpp"
#include "dyn_collision_avoid/msg/sensors_msg.hpp"
#include "dyn_collision_avoid/msg/robot_msg.hpp"

// Utils Classes
#include <DynamicCollisionAvoidance/utils/KDLhelper.h>
#include <DynamicCollisionAvoidance/utils/algebra.h>
#include <DynamicCollisionAvoidance/utils/utilities.h>
#include <DynamicCollisionAvoidance/utils/logger.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
/**
 * @brief  DynamicCollisionAvoidance class is aimed to generate local modification of the 
 *         robot trajectories in order to avoid dynamical obstacles (not limited to) 
 *         based on measurements data (distance)
 */
class DynamicCollisionAvoidanceManager
{

public:

    /////////////// COSTRUCTOR ////////////////////
    /* costructor of the class */
    DynamicCollisionAvoidanceManager( std::string ns = "" );
    /////////// COMPUTATION functions ////////////
    /**
    * callback of the ROS Node
    * @return void
    */
    void timer_callback();
    /**
    * spin function
    * @return void
    */
    void spin();
    /////////////// DISTRUCTOR ////////////////////
    /* distructor of the class */
    ~DynamicCollisionAvoidanceManager();

private:

    /**
     * init ROS Node
     * @return void
     */
    void initROSNode();
    /**
     * load ros node params
     * @return void
     */
    void loadParam();
    /**
     * check if the collision is going to happen
     * @return void
     */
    void checkCollision();
    /**
     * if the collision is going to happen this function is triggered and the relavite variation to avoid the collision is determined
     * @return void
     */
    Eigen::VectorXd computeVariation();
    /**
     * the calculated variation is then planned with this function
     * @return void
     */
    void planCorrection();
    /**
     * Inverse Kinematics
     * @return Eigen::VectorXd
     */
    Eigen::VectorXd IK(Eigen::VectorXd t_ref);
    /**
     *  subscribe task target : assumed point to point trajectory
     *  @return void
     */
    void motionSubscriberCallback(const dyn_collision_avoid::msg::MotionMsg::SharedPtr msg);
    /**
     *  subscribe data sensors
     *  @return void
     */
    void sensorsSubscriberCallback(const dyn_collision_avoid::msg::SensorsMsg::SharedPtr msg);
    /**
     * create robot message
     * @return void
     */
    void composeRobotMsg();
    /**
     * log data
     * @return void
     */
    void log();
    
    //  ROS ----------------------------------------------------------------
    rclcpp::Node::SharedPtr _nh; /* ROSE node handle */
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Subscription<dyn_collision_avoid::msg::MotionMsg>::SharedPtr _motionSubscriber; /* ROS topic object: target trajectory subscriber */
    rclcpp::Subscription<dyn_collision_avoid::msg::SensorsMsg>::SharedPtr _sensorsSubscriber; /* ROS topic object: data measurements subscriber */
    rclcpp::Publisher<dyn_collision_avoid::msg::RobotMsg>::SharedPtr _robotPublisher; /* ROS topic object: robot control references publisher */
    std::string _topicSensorsSub;
    std::string _topicMotionSub;
    std::string _topicRobotPub;
    double _rate;

    //  Utilities ----------------------------------------------------------
    Algebra _algebra; // algebra functions container
    Utilities _utils; // utils
    LoggerL _loggerL; // data logger
    std::string _logPath;

    KDLHelper _kdl; // kdl
    std::string _robotURDFModelPath;
    std::string _base;
    std::string _endEffector;

    // Data ----------------------------------------------------------------
    // Sensors
    Eigen::VectorXd _distances;
    Eigen::VectorXd _sFrame; // sensors frame
    Eigen::VectorXd _sensors;
    std::vector<std::string> _sFrameName;
    double _threshold;
    int _ns;

    // Cartesian Space
    int _nc = 6;
    Eigen::VectorXd _target_pose;
    Eigen::VectorXd _target_pose_i;
    Eigen::VectorXd _target_pose_delta;
    Eigen::VectorXd _target_pose_delta_i;
    Eigen::VectorXd _target_pose_delta_prev;
    Eigen::VectorXd _target_pose_i_received;
    
    // Joint Space
    int _nj;
    Eigen::VectorXd _qref; // joints variable

    double _ref_time;
    double _it_time;
    double _dt;

    dyn_collision_avoid::msg::RobotMsg _robotMsg;
    int _idx;

    // status --------------------------------------------------------
    bool _subscribedData;
    bool _subscribedTarget;
    bool _planCorrection;
    bool _collisionFound;
};

#endif //
