#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <omni_msgs/OmniButtonEvent.h>
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

            // Joint states publisher for shadow robot
            scorbot_shadow_pub_ = nh_.advertise<sensor_msgs::JointState>("shadow_joint_states", 10);
            // Fill joint state message for shadow robot
            std::size_t shadow_dof = 5;
            scorbot_shadow_js_.position.resize(shadow_dof, 0.0);
            scorbot_shadow_js_.velocity.resize(shadow_dof, 0.0);
            scorbot_shadow_js_.effort.resize(shadow_dof, 0.0);
            scorbot_shadow_js_.name.reserve(shadow_dof);
            scorbot_shadow_js_.name.push_back("base");
            scorbot_shadow_js_.name.push_back("shoulder");
            scorbot_shadow_js_.name.push_back("elbow");
            scorbot_shadow_js_.name.push_back("pitch");
            scorbot_shadow_js_.name.push_back("roll");

            // Robot command publisher
            scorbot_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/scorbot/base_controller/command", 10);
            cmd_.data = 0.0;

            // Joint mapping functions
            joint_map_["base"] = boost::make_shared<LinearJointMapping>(1.0, 0.0);

        };

        void hapticButtonStateCallback(const omni_msgs::OmniButtonEvent &haptic_button)
        {
            publish_cmd_ = (bool) haptic_button.grey_button;
        }

        void hapticJointStateCallback(const sensor_msgs::JointStateConstPtr &haptic_joint_state)
        {
            // Estimate mapping joint targets
            JointMap::iterator jm = joint_map_.find("base");
            if (jm != joint_map_.end())
            {
                scorbot_shadow_js_.position[0] = jm->second->map(haptic_joint_state->position[0]);
                ROS_DEBUG_STREAM_THROTTLE(0.1, scorbot_shadow_js_.position[0]);
            }

            // Publish shadow robot joint state
            scorbot_shadow_js_.header.stamp = ros::Time::now();
            scorbot_shadow_pub_.publish(scorbot_shadow_js_);

            // Update robot command
            cmd_.data = scorbot_shadow_js_.position[0];
            if (publish_cmd_) scorbot_cmd_pub_.publish(cmd_);
        }




    private:
        ros::NodeHandle nh_, nh_priv_;
        ros::Subscriber haptic_js_sub_;
        ros::Subscriber haptic_btn_sub_;
        ros::Publisher scorbot_shadow_pub_;
        ros::Publisher scorbot_cmd_pub_;
        sensor_msgs::JointState scorbot_shadow_js_;
        bool publish_cmd_;
        std_msgs::Float64 cmd_;
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
