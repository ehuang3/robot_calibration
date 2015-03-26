#include <robot_calibration/UrdfLoader.h>
#include <ros/package.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>
#include <boost/xpressive/xpressive.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>

namespace robot_calibration {

    typedef boost::shared_ptr<urdf::ModelInterface> ModelInterfacePtr;
    typedef boost::shared_ptr<urdf::Link> UrdfLinkPtr;
    typedef boost::shared_ptr<urdf::Joint> UrdfJointPtr;

    bool ExploreLink(Robotd& robot, ModelInterfacePtr model, UrdfLinkPtr link, Linkd& parent);

    bool CopyLinkProperties(Linkd& link, UrdfLinkPtr ulink);

    bool CopyJointProperties(Jointd& joint, UrdfJointPtr ujoint);

    bool ParseUriRef(const string& uriRef,
                     string& scheme,
                     string& authority,
                     string& path,
                     string& query,
                     string& fragment) {
        // Use boost xpressive instead of pcrecpp.
        using namespace boost::xpressive;
        sregex rex = sregex::compile("^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?");
        smatch what;
        if (regex_match(uriRef, what, rex))
        {
            scheme = what[2];
            authority = what[4];
            path = what[5];
            query = what[7];
            fragment = what[9];
            return true;
        }
        return false;
    }

    bool LoadUrdf(Robotd& robot, const std::string& urdf_uri) {
        // Parse URI for file location.
        std::string scheme, authority, path, _q, _f;
        if (!ParseUriRef(urdf_uri, scheme, authority, path, _q, _f)) {
            ROS_ERROR("Failed to parse URI: %s", urdf_uri.c_str());
            return false;
        }
        if (scheme == "package") {
            std::string package_path = ros::package::getPath(authority);
            path = package_path + path;
        }

        // Read in URDF file.
        std::string xml_model_string;
        std::fstream xml_file(path.c_str(), std::fstream::in);
        if (!xml_file.good()) {
            ROS_ERROR("Could not find URDF file: %s", path.c_str());
            return false;
        }
        while (xml_file.good()) {
            std::string line;
            std::getline(xml_file, line);
            xml_model_string += line + "\n";
        }
        xml_file.close();

        return LoadUrdfString(robot, xml_model_string);
    }

    bool LoadUrdfString(Robotd& robot, const std::string& urdf_string) {
        // Parse URDF model.
        ModelInterfacePtr model;
        model = urdf::parseURDF(urdf_string);

        // Create and add root link.
        UrdfLinkPtr urdf_link = model->root_link_;
        if (robot.getNumLinks() > 0) {
            ROS_ERROR("Links already exist for robot: %s", robot.getName().c_str());
            return false;
        }
        Linkd* link = new Linkd;
        CopyLinkProperties(robot.getRootLink(), urdf_link);
        robot->setRootLink(link);

        // Set robot name.
        Robotd::State& robot_state = robot.getState();
        robot_state.robot_name = model->getName();

        return ExploreLink(robot, model, urdf_link, link);
    }

    bool ExploreLink(Robotd& robot,
                     ModelInterfacePtr model,
                     UrdfLinkPtr link,
                     Linkd& parent) {
        bool success = true;

        for (size_t i = 0; i < link->child_joints.size(); i++) {
            UrdfJointPtr urdf_joint = link->child_joints[i];

        }
    }

    bool CopyLinkProperties(Linkd& link, UrdfLinkPtr ulink) {

    }

    bool CopyJointProperties(Jointd& joint, UrdfJointPtr ujoint) {
        // Get a reference to the joint state.
        Jointd::State& joint_state = joint.getState();

        joint_state.joint_name = ujoint->name;

        Eigen::Vector3d t(ujoint->parent_to_joint_origin_transform.position.x,
                          ujoint->parent_to_joint_origin_transform.position.y,
                          ujoint->parent_to_joint_origin_transform.position.z);
        Eigen::Quaterniond q(ujoint->parent_to_joint_origin_transform.rotation.w,
                             ujoint->parent_to_joint_origin_transform.rotation.x,
                             ujoint->parent_to_joint_origin_transform.rotation.y,
                             ujoint->parent_to_joint_origin_transform.rotation.z);
        joint_state.T_origin.translation() = t;
        joint_state.T_origin.linear() = q;

        switch (ujoint->type) {
        case urdf::Joint::REVOLUTE: joint_state.fixed = false; break;
        case urdf::Joint::FIXED:    joint_state.fixed = true;  break;
        default:
            ROS_ERROR("Joint type %d not supported for %s", ujoint->type, joint.getName().c_str());
            return false;
        }

       // https://github.com/mxgrey/akin/blob/master/utils/urdfAkin/src/urdfParsing.cpp#L55
    }


}
