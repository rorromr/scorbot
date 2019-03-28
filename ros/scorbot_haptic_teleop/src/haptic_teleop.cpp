#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <omni_msgs/OmniButtonEvent.h>
#include <omni_msgs/OmniFeedback.h>
#include <std_msgs/Float64.h>

namespace scorbot
{
    class LinearJointMapping
    {
    public:
        LinearJointMapping():
            scale_(0.0),
            offset_(0.0)
        {};

        LinearJointMapping(const double scale, const double offset):
                scale_(scale),
                offset_(offset)
        {};

        double map(const double joint) const
        {
            return scale_*joint + offset_;
        }

    private:
        double scale_;
        double offset_;
    };
    typedef boost::shared_ptr<LinearJointMapping> LinearJointMappingPtr;

    class HapticTeleop
    {
    public:
        HapticTeleop():
        nh_(),
        nh_priv_("~"),
        name_("haptic_teleop"),
        publish_cmd_(false)
        {
            // Haptic joint states
            haptic_js_sub_ = nh_.subscribe("haptic_joint_states", 10, &HapticTeleop::hapticJointStateCallback, this);
            // Haptic buttons
            haptic_btn_sub_ = nh_.subscribe("haptic_button", 10, &HapticTeleop::hapticButtonStateCallback, this);
            // Haptic force
            haptic_force_pub_ = nh_.advertise<omni_msgs::OmniFeedback>("haptic_force", 10);
            // Real robot joint state
            robot_js_sub_ = nh_.subscribe("joint_states", 10, &HapticTeleop::robotJointStateCallback, this);

            // Joint states publisher for shadow robot
            scorbot_shadow_pub_ = nh_.advertise<sensor_msgs::JointState>("shadow_joint_states", 10);
            // Fill joint state message for shadow robot
            std::size_t shadow_dof = 7; // 5 DOF for joints + 2 DOF for gripper
            scorbot_shadow_js_.position.resize(shadow_dof, 0.0);
            scorbot_shadow_js_.velocity.resize(shadow_dof, 0.0);
            scorbot_shadow_js_.effort.resize(shadow_dof, 0.0);
            scorbot_shadow_js_.name.reserve(shadow_dof);
            scorbot_shadow_js_.name.push_back("base");
            scorbot_shadow_js_.name.push_back("shoulder");
            scorbot_shadow_js_.name.push_back("elbow");
            scorbot_shadow_js_.name.push_back("pitch");
            scorbot_shadow_js_.name.push_back("roll");
            scorbot_shadow_js_.name.push_back("gripper_finger_right_joint");
            scorbot_shadow_js_.name.push_back("gripper_finger_left_joint");

            // Robot command publisher
            scorbot_base_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/scorbot/base_controller/command", 10);
            scorbot_shoulder_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/scorbot/shoulder_controller/command", 10);
            scorbot_elbow_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/scorbot/elbow_controller/command", 10);
            scorbot_pitch_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/scorbot/pitch_controller/command", 10);
            scorbot_roll_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/scorbot/roll_controller/command", 10);
            
            cmd_.data = 0.0;


            // Joint mapping functions
            joint_map_["base"] = boost::make_shared<LinearJointMapping>(1.0, 0.0);
            joint_map_["shoulder"] = boost::make_shared<LinearJointMapping>(-1.0, 0.3);
            joint_map_["elbow"] = boost::make_shared<LinearJointMapping>(1.0, -1.0);
            joint_map_["pitch"] = boost::make_shared<LinearJointMapping>(0.5, -1.0);
            joint_map_["roll"] = boost::make_shared<LinearJointMapping>(0.5, -3.14);
        };

        void hapticButtonStateCallback(const omni_msgs::OmniButtonEvent &haptic_button)
        {
            publish_cmd_ = (bool) haptic_button.grey_button;
        }

        std::size_t getJointIndex(const std::vector<std::string> &joint_names, const std::string &name)
        {
            for(std::size_t i = 0; i < joint_names.size(); i++)
            {
                if(joint_names[i] == name)
                    return i;
            }
        }

        void robotJointStateCallback(const sensor_msgs::JointStateConstPtr &robot_joint_state)
        {
            // Index
            std::size_t robot_idx = 0;
            std::size_t shadow_idx = 0;
            // Base
            robot_idx = getJointIndex(robot_joint_state->name, "base");
            shadow_idx = getJointIndex(scorbot_shadow_js_.name, "base");
            force_cmd_.force.x = 10.0*(robot_joint_state->position[robot_idx] - scorbot_shadow_js_.position[shadow_idx]);
            ROS_DEBUG_STREAM("robot base: " << robot_joint_state->position[robot_idx] << " | shadow base: " << scorbot_shadow_js_.position[shadow_idx]
                << " | base force " << force_cmd_.force.x);
            // Shoulder
            robot_idx = getJointIndex(robot_joint_state->name, "shoulder");
            shadow_idx = getJointIndex(scorbot_shadow_js_.name, "shoulder");
            force_cmd_.force.y = -20.0*(robot_joint_state->position[robot_idx] - scorbot_shadow_js_.position[shadow_idx]);
            ROS_DEBUG_STREAM("robot shoulder: " << robot_joint_state->position[robot_idx] << " | shadow shoulder: " << scorbot_shadow_js_.position[shadow_idx]
                << " | shoulder force " << force_cmd_.force.y);
            // Elbow
            robot_idx = getJointIndex(robot_joint_state->name, "elbow");
            shadow_idx = getJointIndex(scorbot_shadow_js_.name, "elbow");
            force_cmd_.force.z = 20.0*(robot_joint_state->position[robot_idx] - scorbot_shadow_js_.position[shadow_idx]);
            ROS_DEBUG_STREAM("robot elbow: " << robot_joint_state->position[robot_idx] << " | shadow elbow: " << scorbot_shadow_js_.position[shadow_idx]
                << " | elbow force " << force_cmd_.force.y);

            
            haptic_force_pub_.publish(force_cmd_);
        }

        void hapticJointStateCallback(const sensor_msgs::JointStateConstPtr &haptic_joint_state)
        {
            // Estimate mapping joint targets

            // Base
            JointMap::iterator jm = joint_map_.find("base");
            if (jm != joint_map_.end())
            {
                scorbot_shadow_js_.position[0] = jm->second->map(haptic_joint_state->position[0]);
                ROS_DEBUG_STREAM_THROTTLE(0.1, scorbot_shadow_js_.position[0]);
            }

            // Shoulder
            // Estimate mapping joint targets
            jm = joint_map_.find("shoulder");
            if (jm != joint_map_.end())
            {
                scorbot_shadow_js_.position[1] = jm->second->map(haptic_joint_state->position[1]);
                ROS_DEBUG_STREAM_THROTTLE(0.1, scorbot_shadow_js_.position[1]);
            }

            // Elbow
            // Estimate mapping joint targets
            jm = joint_map_.find("elbow");
            if (jm != joint_map_.end())
            {
                scorbot_shadow_js_.position[2] = jm->second->map(haptic_joint_state->position[2]);
                ROS_DEBUG_STREAM_THROTTLE(0.1, scorbot_shadow_js_.position[2]);
            }

            // Pitch
            // Estimate mapping joint targets
            jm = joint_map_.find("pitch");
            if (jm != joint_map_.end())
            {
                // Use joint 4 from Phantom Omni
                scorbot_shadow_js_.position[3] = jm->second->map(haptic_joint_state->position[4]);
                ROS_DEBUG_STREAM_THROTTLE(0.1, scorbot_shadow_js_.position[3]);
            }

            // Pitch
            // Estimate mapping joint targets
            jm = joint_map_.find("roll");
            if (jm != joint_map_.end())
            {
                // Use joint 4 from Phantom Omni
                scorbot_shadow_js_.position[4] = jm->second->map(haptic_joint_state->position[3]);
                ROS_DEBUG_STREAM_THROTTLE(0.1, scorbot_shadow_js_.position[4]);
            }

            // Publish shadow robot joint state
            scorbot_shadow_js_.header.stamp = ros::Time::now();
            scorbot_shadow_pub_.publish(scorbot_shadow_js_);

            // Update robot command
            cmd_.data = scorbot_shadow_js_.position[0];
            if (publish_cmd_)
            {
                scorbot_base_cmd_pub_.publish(cmd_);
            }

            cmd_.data = scorbot_shadow_js_.position[1];
            if (publish_cmd_)
            {
                scorbot_shoulder_cmd_pub_.publish(cmd_);
            }

            cmd_.data = scorbot_shadow_js_.position[2];
            if (publish_cmd_)
            {
                scorbot_elbow_cmd_pub_.publish(cmd_);
            }

            cmd_.data = scorbot_shadow_js_.position[3];
            if (publish_cmd_)
            {
                scorbot_pitch_cmd_pub_.publish(cmd_);
            }

            cmd_.data = scorbot_shadow_js_.position[4];
            if (publish_cmd_)
            {
                scorbot_roll_cmd_pub_.publish(cmd_);
            }
        }




    private:
        ros::NodeHandle nh_, nh_priv_;
        ros::Subscriber haptic_js_sub_;
        ros::Subscriber haptic_btn_sub_;
        ros::Subscriber robot_js_sub_;
        ros::Publisher haptic_force_pub_;
        ros::Publisher scorbot_shadow_pub_;
        ros::Publisher scorbot_base_cmd_pub_;
        ros::Publisher scorbot_shoulder_cmd_pub_;
        ros::Publisher scorbot_elbow_cmd_pub_;
        ros::Publisher scorbot_pitch_cmd_pub_;
        ros::Publisher scorbot_roll_cmd_pub_;
        sensor_msgs::JointState scorbot_shadow_js_;
        bool publish_cmd_;
        std_msgs::Float64 cmd_;
        omni_msgs::OmniFeedback force_cmd_;
        std::string name_;
        typedef std::map<std::string, LinearJointMappingPtr> JointMap;
        JointMap joint_map_;
    };

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scorbot_haptic_teleop");

    ROS_INFO("Init");
    scorbot::HapticTeleop teleop;

    // Wait until shutdown signal recieved
    ros::spin();

    return 0;
}
