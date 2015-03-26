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
#include <boost/shared_ptr.hpp>
#include <camera_calib/Camera.h>
#include <camera_calib/Marker.h>
#include <camera_calib/CalibrateRobot.h>


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
        typedef Eigen::Transform<T,3,Isometry> IsometryT;
        typedef Eigen::Matrix<T,3,1> Vector3T;

        struct State {
            std::string joint_name;     // The joint's name.
            LinkT*      parent;         // The parent link of this joint.
            LinkT*      child;          // The child link of this joint.
            RobotT*     robot;          // The robot this joint belongs to.
            IsometryT   T_origin;       // The transform from parent link to child link at joint angle zero.
            Vector3T    axis;           // The joint axis (defined the child link frame).
            JointT*     mimic;          // The joint mimicking this joint.
            bool        fixed;          // The boolean signifying whether the joint is fixed or not.
            T           angle;          // The joint angle (fixed for each measurement).
            T           angle_offset;   // The parameter defining the joint angle offset to the real zero.
            Vector3T    axis_offset;    // The parameter defining the joint axis offset to the real axis.
            IsometryT   T_cache;        // The transform from parent link to child link at these parameter values.
            bool        dirty;          // The boolean signifying whether the cached transform requires updating.
            IsometryT   T_global;       // The global transform to the child link for the given parameter values.
        };

        AutoDiffJoint() {
        }

        ~AutoDiffJoint() {
        }

        AutoDiffJoint(const AutoDiffJoint<double>* joint) {
        }

        // Disable copy constructor and assignment operator.
        AutoDiffJoint(const AutoDiffJoint<T>&) = delete;
        AutoDiffJoint<T>& operator=(const AutoDiffJoint<T>&) = delete;

        const State& getState() const {
            return _state;
        }

        State& getState() {
            return _state;
        }

        bool isDirty() {
            return _state.dirty;
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
            // If the cached transform is dirty, recompute it with the
            // new parameters.
            if (_state.dirty) {
                IsometryT T_joint = _state.T_origin;

                // Translate the parent to child frame by the axis offset.
                T_joint.translate(_state.axis_offset);

                // Rotate about the joint axis by joint angle plus joint offset.
                T_joint.rotate(AngleAxisT(_state.axis, _state.angle + _state.angle_offset));

                // Cache new parent to child frame transform.
                _state.T_cache = T_joint;

                _state.dirty = false;
            }

            // Compute the global transform.
            const IsometryT& T_parent = _state.parent->getGlobalTransform();
            _state.T_global = _state.T_cache * T_parent;

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
        typedef Eigen::Transform<T,3,Isometry> IsometryT;
        typedef Eigen::Matrix<T,3,1> Vector3T;

        struct State {
            std::string  link_name;     // The link's name.
            RobotT*      robot;         // The robot this link belongs to.
            JointT*      parent;        // The joint this link is a child of.
            JointVectorT children;      // The joints this link is parent to.
            IsometryT    T_origin;      // The inertial frame origin of this link.
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

        void setDirty(bool dirty = true) {
            _state.dirty = dirty;
        }

        const IsometryT& getGlobalTransform() const
        {
            if (_state.parent == NULL)
                return IsometryT::Identity();
            if (_state.dirty)
                ROS_ERROR("Requesting dirty transform for %s", _state.link_name.c_str());
            return _state.T_global;
        }

        void update() {
            // Compute the global transform.
            _state.T_global = _state.parent->getGlobalTransform();
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
        typedef Eigen::Transform<T,3,Isometry> IsometryT;
        typedef Eigen::Matrix<T,3,1> Vector3T;

        struct State {
            std::string    robot_name;

            LinkT* root;

            JointVectorT   joints;
            LinkVectorT    links;
            JointVectorT   active_joints;

            JointNameMapT  joint_name_map;
            LinkNameMapT   link_name_map;
            JointIndexMapT joint_index_map;
            LinkIndexMapT  link_index_map;

        };

        AutoDiffRobot() {
        }

        AutoDiffRobot(const AutoDiffRobot<double>* robot) {
        }

        // Disable copy constructor and assignment operator.
        AutoDiffLink(const AutoDiffLink<T>&) = delete;
        AutoDiffLink<T>& operator=(const AutoDiffLink<T>&) = delete;

        const std::string& getName() const {
            return _state.robot_name;
        }

        int getNumLinks() const {
            return _state.links.size();
        }

        const State& getState() const {
            return _state;
        }

        State& getState() {
            return _state;
        }

        void update() {
            _state.root->update();
        }

    protected:
        State _state;
    };

    typedef AutoDiffRobot<double> Robotd;
    typedef AutoDiffJoint<double> Jointd;
    typedef AutoDiffLink<double>  Linkd;

    Robotd* LoadRobotd();
    Jointd* LoadJointd();
    Linkd*  LoadLinkd();
}
