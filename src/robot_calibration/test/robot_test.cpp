/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Georgia Tech Research Corporation
 *  All rights reserved.
 *
 *  Author(s): Eric Huang <ehuang@gatech.edu>
 *  Georgia Tech Socially Intelligent Machines Lab
 *  Under Direction of Prof. Andrea Thomaz <athomaz@cc.gatech.edu>
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <robot_calibration/robot.h>
#include <robot_calibration/urdf_loader.h>
#include <dart/utils/utils.h>
#include <dart/dynamics/dynamics.h>


TEST(ROBOT, LOAD_URDF)
{
    robot_calibration::Robotd* robot = new robot_calibration::Robotd;
    robot_calibration::LoadUrdf(robot, "package://robot_calibration/config/example.urdf", true);
    robot->update();
}

TEST(ROBOT, SET_JOINT_ANGLES)
{
    robot_calibration::Robotd* robot = new robot_calibration::Robotd;
    robot_calibration::LoadUrdf(robot, "package://robot_calibration/config/example.urdf");
    Eigen::VectorXd upper, lower;
    robot->getJointLimits(upper, lower);
    Eigen::VectorXd angles = Eigen::VectorXd::Random(robot->getNumActiveJoints());
    for (int i = 0; i < angles.size(); i++) {
        angles[i] = angles[i] * (upper[i] - lower[i]) / 2 + (upper[i] - lower[i]) / 2 + lower[i];
    }
    robot->setJointAngles(angles.data());
    robot->update();
    for (int i = 0; i < angles.size(); i++) {
        angles[i] = upper[i];
    }
    robot->setJointAngles(angles.data());
    robot->update();
}

TEST(ROBOT, GET_GLOBAL_TRANSFORM)
{
    robot_calibration::Robotd* robot = new robot_calibration::Robotd;
    robot_calibration::LoadUrdf(robot, "package://robot_calibration/config/example.urdf");
    Eigen::VectorXd upper, lower;
    robot->getJointLimits(upper, lower);
    Eigen::VectorXd angles = Eigen::VectorXd::Random(robot->getNumActiveJoints());
    // for (int i = 0; i < angles.size(); i++) {
    //     angles[i] = angles[i] * (upper[i] - lower[i]) / 2 + (upper[i] - lower[i]) / 2 + lower[i];
    // }
    angles.setZero();
    robot->setJointAngles(angles.data());
    robot->update();
    std::vector<robot_calibration::Jointd*> joints = robot->getJoints();
    // for (int i = 0; i < joints.size(); i++) {
    //     std::cout << joints[i]->getName() << std::endl;
    //     std::cout << joints[i]->getGlobalTransform().matrix() << std::endl;
    // }
}

TEST(ROBOT, COMPARE_WITH_DART)
{
    std::string package_path = "/home/ehuang/catkin_ws/src/apc_ros/";
    std::string urdf_path    = "/home/ehuang/catkin_ws/src/robot_calibration/config/example.urdf";
    dart::utils::DartLoader dart_loader;
    dart_loader.setPackageDirectory(package_path);
    dart::dynamics::Skeleton* dart_robot = dart_loader.parseSkeleton(urdf_path);
    dart_robot->init();
    dart_robot->computeForwardKinematics();
    int num_dofs = dart_robot->getNumDofs();
    // std::cout << num_dofs << std::endl;
    // for (int i = 0; i < num_dofs; i++)
    // {
    //     std::cout << dart_robot->getJoint(i)->getName() << std::endl;
    //     std::cout << dart_robot->getJoint(i)->getChildBodyNode()->getTransform().matrix() << std::endl;
    // }

    robot_calibration::Robotd* robot = new robot_calibration::Robotd;
    robot_calibration::LoadUrdf(robot, "package://robot_calibration/config/example.urdf");
    Eigen::VectorXd upper, lower;
    robot->getJointLimits(upper, lower);
    Eigen::VectorXd angles = Eigen::VectorXd::Random(robot->getNumActiveJoints());
    // for (int i = 0; i < angles.size(); i++) {
    //     angles[i] = angles[i] * (upper[i] - lower[i]) / 2 + (upper[i] - lower[i]) / 2 + lower[i];
    // }
    angles.setZero();
    robot->setJointAngles(angles.data());
    robot->update();
    std::vector<robot_calibration::Jointd*> joints = robot->getActiveJoints();

    for (int i = 0; i < num_dofs; i++) {
        std::string joint_name = dart_robot->getJoint(i)->getName();
        Eigen::Matrix4d T_dart = dart_robot->getJoint(i)->getChildBodyNode()->getTransform().matrix();
        // std::cout << joint_name << std::endl;
        if (!robot->getJoint(joint_name))
            continue;
        Eigen::Matrix4d T_joint = robot->getJoint(joint_name)->getGlobalTransform().matrix();
        for (int r = 0; r < T_dart.rows(); r++)
            for (int c = 0; c < T_dart.cols(); c++)
                ASSERT_NEAR(T_dart(r,c), T_joint(r,c), 1e-7);
    }

    angles.setRandom();
    for (int i = 0; i < angles.size(); i++) {
        angles[i] = angles[i] * (upper[i] - lower[i]) / 2 + (upper[i] - lower[i]) / 2 + lower[i];
    }
    robot->setJointAngles(angles.data());
    robot->update();
    for (int i = 0; i < joints.size(); i++) {
        std::string joint_name = joints[i]->getName();
        dart_robot->getJoint(joint_name)->getDof(0)->setPosition(angles[i]);
    }
    dart_robot->computeForwardKinematics();

    for (int i = 0; i < num_dofs; i++) {
        std::string joint_name = dart_robot->getJoint(i)->getName();
        Eigen::Matrix4d T_dart = dart_robot->getJoint(i)->getChildBodyNode()->getTransform().matrix();

        if (!robot->getJoint(joint_name))
            continue;
        Eigen::Matrix4d T_joint = robot->getJoint(joint_name)->getGlobalTransform().matrix();

        // std::cout << joint_name << std::endl;
        // std::cout << T_dart.matrix() << std::endl;
        // std::cout << T_joint.matrix() << std::endl;

        for (int r = 0; r < T_dart.rows(); r++)
            for (int c = 0; c < T_dart.cols(); c++)
                ASSERT_NEAR(T_dart(r,c), T_joint(r,c), 1e-7);
    }

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "robot_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
