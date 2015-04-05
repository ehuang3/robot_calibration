#include <robot_calibration/urdf_loader.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>
#include <boost/xpressive/xpressive.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <geometric_shapes/shape_operations.h>


namespace robot_calibration {

    typedef boost::shared_ptr<urdf::ModelInterface> ModelInterfacePtr;
    typedef boost::shared_ptr<urdf::Link> UrdfLinkPtr;
    typedef boost::shared_ptr<urdf::Joint> UrdfJointPtr;

    bool ExploreLink(Robotd* robot, ModelInterfacePtr model, UrdfLinkPtr link, Linkd* parent, bool load_meshes);

    bool CopyLinkProperties(Linkd* link, UrdfLinkPtr ulink, bool load_meshes);

    shapes::ShapePtr ConstructShape(const urdf::Geometry *geom);

    bool CopyJointProperties(Jointd* joint, UrdfJointPtr ujoint);

    bool ParseUriRef(const std::string& uriRef,
                     std::string& scheme,
                     std::string& authority,
                     std::string& path,
                     std::string& query,
                     std::string& fragment) {
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

    bool LoadUrdf(Robotd* robot, const std::string& urdf_uri, bool load_meshes) {
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

        return LoadUrdfString(robot, xml_model_string, load_meshes);
    }

    bool LoadUrdfString(Robotd* robot, const std::string& urdf_string, bool load_meshes) {
        // Parse URDF model.
        ModelInterfacePtr model;
        model = urdf::parseURDF(urdf_string);

        // Create and add root link.
        UrdfLinkPtr urdf_link = model->root_link_;
        if (robot->getNumLinks() > 0) {
            ROS_ERROR("Links already exist for robot: %s", robot->getName().c_str());
            return false;
        }
        Linkd* root_link = new Linkd;
        CopyLinkProperties(root_link, urdf_link, load_meshes);
        robot->setRootLink(root_link);

        // Set robot name.
        robot->setName(model->getName());

        // Build robot model.
        bool success = ExploreLink(robot, model, urdf_link, root_link, load_meshes);

        // Update all transforms.
        robot->update();

        return success;
    }

    bool ExploreLink(Robotd* robot,
                     ModelInterfacePtr model,
                     UrdfLinkPtr urdf_link,
                     Linkd* parent,
                     bool load_meshes) {
        bool success = true;

        // Create all child links and add them to the robot. We want
        // to create the child links before creating the child joints.
        for (size_t i = 0; i < urdf_link->child_links.size(); i++) {
            Linkd* child = new Linkd;
            CopyLinkProperties(child, urdf_link->child_links[i], load_meshes);
            robot->addLink(child);
        }

        // Create all child joints and add them to the robot.
        for (size_t i = 0; i < urdf_link->child_joints.size(); i++) {
            Jointd* joint = new Jointd;
            CopyJointProperties(joint, urdf_link->child_joints[i]);
            Linkd* child = robot->getLink(urdf_link->child_links[i]->name);
            robot->addJoint(joint, parent, child);
        }

        // Traverse child links and add their descendants to the
        // kinematic tree.
        for (size_t i = 0; i < urdf_link->child_links.size(); i++) {
            Linkd* child = robot->getLink(urdf_link->child_links[i]->name);
            ExploreLink(robot, model, urdf_link->child_links[i], child, load_meshes);
        }

        return success;
    }

    bool CopyLinkProperties(Linkd* link, UrdfLinkPtr ulink, bool load_meshes) {
        Linkd::State& link_state = link->getState();

        link_state.link_name = ulink->name;

        if (ulink->inertial) {
            Eigen::Vector3d t(ulink->inertial->origin.position.x,
                              ulink->inertial->origin.position.y,
                              ulink->inertial->origin.position.z);
            Eigen::Quaterniond q(ulink->inertial->origin.rotation.w,
                                 ulink->inertial->origin.rotation.x,
                                 ulink->inertial->origin.rotation.y,
                                 ulink->inertial->origin.rotation.z);
            link_state.T_inertial.translation() = t;
            link_state.T_inertial.linear() = q.matrix();
        }

        if (load_meshes)
            for (int i = 0; i < ulink->visual_array.size(); i++) {
                Eigen::Isometry3d T_shape;
                Eigen::Vector3d t(ulink->visual_array[i]->origin.position.x,
                                  ulink->visual_array[i]->origin.position.y,
                                  ulink->visual_array[i]->origin.position.z);
                Eigen::Quaterniond q(ulink->visual_array[i]->origin.rotation.w,
                                     ulink->visual_array[i]->origin.rotation.x,
                                     ulink->visual_array[i]->origin.rotation.y,
                                     ulink->visual_array[i]->origin.rotation.z);
                T_shape.translation() = t;
                T_shape.linear() = q.matrix();
                link_state.T_shapes.push_back(T_shape);

                shapes::ShapePtr shape = ConstructShape(ulink->visual_array[i]->geometry.get());
                link_state.shapes.push_back(shape);
            }

        return true;
    }

    shapes::ShapePtr ConstructShape(const urdf::Geometry *geom)
    {
        shapes::Shape *result = NULL;
        switch (geom->type)
        {
        case urdf::Geometry::SPHERE:
            result = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
            break;
        case urdf::Geometry::BOX:
        {
            urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
            result = new shapes::Box(dim.x, dim.y, dim.z);
        }
        break;
        case urdf::Geometry::CYLINDER:
            result = new shapes::Cylinder(static_cast<const urdf::Cylinder*>(geom)->radius,
                                          static_cast<const urdf::Cylinder*>(geom)->length);
            break;
        case urdf::Geometry::MESH:
        {
            const urdf::Mesh *mesh = static_cast<const urdf::Mesh*>(geom);
            if (!mesh->filename.empty())
            {
                Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
                shapes::Mesh *m = shapes::createMeshFromResource(mesh->filename, scale);
                result = m;
            }
        }
        break;
        default:
            ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
            break;
        }

        return shapes::ShapePtr(result);
    }

    bool CopyJointProperties(Jointd* joint, UrdfJointPtr ujoint) {
        // Get a reference to the joint state.
        Jointd::State& joint_state = joint->getState();

        joint_state.joint_name = ujoint->name;

        Eigen::Vector3d t(ujoint->parent_to_joint_origin_transform.position.x,
                          ujoint->parent_to_joint_origin_transform.position.y,
                          ujoint->parent_to_joint_origin_transform.position.z);
        Eigen::Quaterniond q(ujoint->parent_to_joint_origin_transform.rotation.w,
                             ujoint->parent_to_joint_origin_transform.rotation.x,
                             ujoint->parent_to_joint_origin_transform.rotation.y,
                             ujoint->parent_to_joint_origin_transform.rotation.z);
        joint_state.T_origin.translation() = t;
        joint_state.T_origin.linear() = q.matrix();

        switch (ujoint->type) {
        case urdf::Joint::REVOLUTE: joint_state.fixed = false; break;
        case urdf::Joint::FIXED:    joint_state.fixed = true;  break;
        default:
            ROS_ERROR("Joint type %d not supported for %s", ujoint->type, ujoint->name.c_str());
            return false;
        }

        joint_state.axis = Eigen::Vector3d(ujoint->axis.x, ujoint->axis.y, ujoint->axis.z);
        if (joint_state.axis.norm() == 0)
            joint_state.axis = Eigen::Vector3d::UnitZ();

        if (ujoint->limits)
        {
            joint_state.upper_limit = ujoint->limits->upper;
            joint_state.lower_limit = ujoint->limits->lower;
        }

        return true;
    }


}
