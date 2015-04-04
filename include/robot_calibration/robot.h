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
#pragma once
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <robot_calibration/exception.h>


namespace robot_calibration
{
    // Forward declarations of auto-diff classes.
    template <typename T> class AutoDiffJoint;
    template <typename T> class AutoDiffLink;
    template <typename T> class AutoDiffRobot;

    template <typename T>
    class AutoDiffJoint {
    public:
        typedef AutoDiffLink<T> LinkT;
        typedef AutoDiffJoint<T> JointT;
        typedef AutoDiffRobot<T> RobotT;
        typedef Eigen::Transform<T,3,Eigen::Isometry> IsometryT;
        typedef Eigen::Matrix<T,2,1> Vector2T;
        typedef Eigen::Matrix<T,3,1> Vector3T;
        typedef Eigen::AngleAxis<T> AngleAxisT;

        struct State {
            std::string joint_name;     // The joint's name.
            LinkT*      parent;         // The parent link of this joint.
            LinkT*      child;          // The child link of this joint.
            RobotT*     robot;          // The robot this joint belongs to.
            IsometryT   T_origin;       // The transform from parent link to child link at joint angle zero.
            Vector3T    axis;           // The joint axis (defined the child link frame).
            JointT*     mimic;          // The joint mimicking this joint.
            bool        fixed;          // The boolean signifying whether the joint is fixed or not.
            T           upper_limit;    // The joint angle upper limit.
            T           lower_limit;    // The joint angle lower limit.
            T           angle;          // The joint angle (fixed for each measurement).
            T           angle_offset;   // The parameter defining the joint angle offset to the real zero.
            Vector3T    axis_offset;    // The parameter defining the joint axis offset to the real axis.
            IsometryT   T_joint;        // The transform from parent link to child link at these parameter values.
            bool        dirty;          // The boolean signifying whether the joint transform requires updating.
            IsometryT   T_global;       // The global transform to the child link for the given parameter values.
        };

        AutoDiffJoint() {
            reset();
        }

        ~AutoDiffJoint() {
        }

        AutoDiffJoint(const AutoDiffJoint<double>* joint) {
        }

        // Disable copy constructor and assignment operator.
        AutoDiffJoint(const AutoDiffJoint<T>&) = delete;
        AutoDiffJoint<T>& operator=(const AutoDiffJoint<T>&) = delete;

        void reset() {
            _state.joint_name = "";
            _state.parent = NULL;
            _state.child = NULL;
            _state.robot = NULL;
            _state.T_origin = IsometryT::Identity();
            _state.axis = Vector3T::UnitZ();
            _state.mimic = NULL;
            _state.fixed = true;
            _state.upper_limit = T(0);
            _state.lower_limit = T(0);
            _state.angle = T(0);
            _state.angle_offset = T(0);
            _state.axis_offset = Vector3T::Zero();
            _state.T_joint = IsometryT::Identity();
            _state.dirty = true;
            _state.T_global = IsometryT::Identity();
        }

        const State& getState() const {
            return _state;
        }

        State& getState() {
            return _state;
        }

        const std::string& getName() {
            return _state.joint_name;
        }

        bool isDirty() {
            return _state.dirty;
        }

        bool setAngle(T angle) {
            if (_state.fixed)
                return false;
            ROBOT_CALIBRATION_ASSERT(angle <= _state.upper_limit,
                                     "Joint %s angle above upper limit %f",
                                     _state.joint_name.c_str(), _state.upper_limit);
            ROBOT_CALIBRATION_ASSERT(angle >= _state.lower_limit,
                                     "Joint %s angle below lower limit %f",
                                     _state.joint_name.c_str(), _state.lower_limit);
            _state.angle = angle;
            _state.dirty = true;
            return true;
        }

        bool setAngleOffset(T* offset) {
            if (_state.fixed)
                return false;
            _state.angle_offset = *offset;
            _state.dirty = true;
            _state.child->setDirty();
            return true;
        }

        bool setAxisOffset(T* offset) {
            if (_state.fixed)
                return false;
            _state.angle_axis[0] = offset[0];
            _state.angle_axis[1] = offset[1];
            _state.angle_axis[2] = offset[2];
            _state.dirty = true;
            _state.child->setDirty();
            return true;
        }

        const IsometryT& getGlobalTransform() const
        {
            if (_state.dirty)
                ROS_ERROR("Requesting dirty transform for %s", _state.joint_name.c_str());
            return _state.T_global;
        }

        void update() {
            // If the jointd transform is dirty, recompute it with the
            // new parameters.
            if (_state.dirty) {
                IsometryT T_joint = _state.T_origin;

                // Translate the parent to child frame by the axis offset.
                T_joint.translate(_state.axis_offset);

                // Rotate about the joint axis by joint angle plus joint offset.
                T_joint.rotate(AngleAxisT(_state.angle + _state.angle_offset, _state.axis));

                // Cache new parent to child frame transform.
                _state.T_joint = T_joint;

                _state.dirty = false;
            }

            // Compute the global transform.
            const IsometryT& T_parent = _state.parent->getGlobalTransform();
            _state.T_global = T_parent * _state.T_joint;
            // _state.T_global = _state.T_joint * T_parent;

            // Update the child.
            _state.child->update();
        }

    protected:
        State _state;
    };

    template <typename T>
    class AutoDiffLink {
    public:
        typedef AutoDiffLink<T> LinkT;
        typedef AutoDiffJoint<T> JointT;
        typedef AutoDiffRobot<T> RobotT;
        typedef std::vector<JointT*> JointVectorT;
        typedef Eigen::Transform<T,3,Eigen::Isometry> IsometryT;
        typedef Eigen::Matrix<T,3,1> Vector3T;

        struct State {
            std::string  link_name;     // The link's name.
            RobotT*      robot;         // The robot this link belongs to.
            JointT*      parent;        // The joint this link is a child of.
            JointVectorT children;      // The joints this link is parent to.
            IsometryT    T_inertial;    // The inertial frame of this link.
            IsometryT    T_global;      // The global transform to this link.
            bool         dirty;         // The boolean signifying whether the global transform requires updating.
        };

        AutoDiffLink() {
        }

        AutoDiffLink(const AutoDiffLink<double>* link) {
        }

        // Disable copy constructor and assignment operator.
        AutoDiffLink(const AutoDiffLink<T>&) = delete;
        AutoDiffLink<T>& operator=(const AutoDiffLink<T>&) = delete;

        const State& getState() const {
            return _state;
        }

        State& getState() {
            return _state;
        }

        const std::string& getName() {
            return _state.link_name;
        }

        void setDirty(bool dirty = true) {
            _state.dirty = dirty;
        }

        IsometryT getGlobalTransform() const
        {
            if (_state.parent == NULL)
                return IsometryT::Identity();
            if (_state.dirty)
                ROS_ERROR("Requesting dirty transform for %s", _state.link_name.c_str());
            return _state.T_global;
        }

        void update() {
            // Compute the global transform.
            if (_state.parent)
                _state.T_global = _state.parent->getGlobalTransform();
            else
                _state.T_global = IsometryT::Identity();
            _state.dirty = false;

            // Update the children.
            for (int i = 0; i < _state.children.size(); i++)
                _state.children[i]->update();
        }

    protected:
        State _state;
    };

    template <typename T>
    class AutoDiffRobot {
    public:
        typedef AutoDiffLink<T> LinkT;
        typedef AutoDiffJoint<T> JointT;
        typedef AutoDiffRobot<T> RobotT;
        typedef std::vector<JointT*> JointVectorT;
        typedef std::vector<LinkT*> LinkVectorT;
        typedef std::map<std::string, JointT*> JointNameMapT;
        typedef std::map<std::string, LinkT*> LinkNameMapT;
        typedef std::map<int, JointT*> JointIndexMapT;
        typedef std::map<int, LinkT*> LinkIndexMapT;
        typedef Eigen::Transform<T,3,Eigen::Isometry> IsometryT;
        typedef Eigen::Matrix<T,3,1> Vector3T;
        typedef Eigen::Matrix<T,Eigen::Dynamic,1> VectorXT;

        struct State {
            std::string    robot_name;
            LinkT*         root_link;
            LinkVectorT    links;
            JointVectorT   joints;
            JointVectorT   active_joints;
            JointNameMapT  joint_name_map;
            LinkNameMapT   link_name_map;
        };

        AutoDiffRobot() {
        }

        AutoDiffRobot(const AutoDiffRobot<double>* robot) {
        }

        // Disable copy constructor and assignment operator.
        AutoDiffRobot(const AutoDiffRobot<T>&) = delete;
        AutoDiffRobot<T>& operator=(const AutoDiffRobot<T>&) = delete;

        const std::string& getName() const {
            return _state.robot_name;
        }

        void setName(const std::string& name) {
            _state.robot_name = name;
        }

        int getNumLinks() const {
            return _state.links.size();
        }

        int getNumActiveJoints() const {
            return _state.active_joints.size();
        }

        LinkT* getRootLink() {
            return _state.root_link;
        }

        void setRootLink(LinkT* link) {
            _state.root_link = link;
            addLink(link);
        }

        LinkT* getLink(const std::string& link_name) {
            if (_state.link_name_map.count(link_name) == 0)
                return NULL;
            return _state.link_name_map[link_name];
        }

        LinkT* getLink(int index) {
            return _state.links[index];
        }

        LinkVectorT getLinks() {
            return _state.links;
        }

        void addLink(LinkT* link) {
            ROBOT_CALIBRATION_ASSERT(std::find(_state.links.begin(), _state.links.end(), link) == _state.links.end(),
                                     "Link %s already exists in robot %s",
                                     link->getName().c_str(), _state.robot_name.c_str());
            typename LinkT::State& link_state = link->getState();
            link_state.robot = this;
            link_state.parent = NULL;
            _state.links.push_back(link);
            _state.link_name_map[link->getName()] = link;
        }

        JointT* getJoint(const std::string& joint_name) {
            if (_state.joint_name_map.count(joint_name) == 0)
                return NULL;
            return _state.joint_name_map[joint_name];
        }

        JointT* getJoint(int index) {
            return _state.joints[index];
        }

        JointVectorT getJoints() {
            return _state.joints;
        }

        JointVectorT getActiveJoints() {
            return _state.active_joints;
        }

        void addJoint(JointT* joint, LinkT* parent, LinkT* child) {
            ROBOT_CALIBRATION_ASSERT(std::find(_state.joints.begin(), _state.joints.end(), joint) == _state.joints.end(),
                                     "Joint %s already exists in robot %s",
                                     joint->getName().c_str(), _state.robot_name.c_str());
            // Fill out joint information.
            typename JointT::State& joint_state = joint->getState();
            joint_state.parent = parent;
            joint_state.child = child;
            joint_state.robot = this;
            _state.joints.push_back(joint);
            _state.joint_name_map[joint->getName()] = joint;
            if (!joint_state.fixed)
                _state.active_joints.push_back(joint);
            // Fill out parent link information.
            typename LinkT::State& parent_state = parent->getState();
            parent_state.children.push_back(joint);
            // Fill out child link information.
            typename LinkT::State& child_state = child->getState();
            child_state.parent = joint;
        }

        const State& getState() const {
            return _state;
        }

        State& getState() {
            return _state;
        }

        void getJointLimits(VectorXT& upper, VectorXT& lower) {
            int num_active = _state.active_joints.size();
            upper.resize(num_active);
            lower.resize(num_active);
            for (int i = 0; i < num_active; i++) {
                typename JointT::State& joint_state = _state.active_joints[i]->getState();
                upper[i] = joint_state.upper_limit;
                lower[i] = joint_state.lower_limit;
            }
        }

        void setJointAngles(const T* angles) {
            for (int i = 0; i < _state.active_joints.size(); i++) {
                _state.active_joints[i]->setAngle(angles[i]);
            }
        }

        void setJointAngles(const std::vector<std::string>& joint_names, const T* angles) {
            for (int i = 0; i < joint_names.size(); i++) {
                JointT* joint = getJoint(joint_names[i]);
                joint->setAngle(angles[i]);
            }
        }

        IsometryT getGlobalTransform(const std::string& frame) {
            if (_state.joint_name_map.find(frame) != _state.joint_name_map.end())
                return _state.joint_name_map[frame]->getGlobalTransform();
            if (_state.link_name_map.find(frame) != _state.link_name_map.end())
                return _state.link_name_map[frame]->getGlobalTransform();
            return IsometryT::Identity();
        }

        void update() {
            _state.root_link->update();
        }

    protected:
        State _state;
    };

    typedef AutoDiffRobot<double> Robotd;
    typedef AutoDiffJoint<double> Jointd;
    typedef AutoDiffLink<double>  Linkd;
}
