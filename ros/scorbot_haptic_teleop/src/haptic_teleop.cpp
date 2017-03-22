#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace scorbot
{
    typedef struct
    {
        double scale;
        double offset;
    } JointMappingParams;

    class HapticTeleop
    {
    public:
        HapticTeleop():
        nh_(),
        nh_priv_("~"),
        name_("haptic_teleop")
        {
            // Haptic joint states
            haptic_sub_ = nh_.subscribe("haptic_joint_states", 10, &HapticTeleop::hapticCallback, this);

        };

        void hapticCallback(const sensor_msgs::JointStateConstPtr& haptic_joint_state)
        {
            double target1 = 1000*haptic_joint_state->position[0];
            ROS_INFO_STREAM_THROTTLE(0.1, target1);
        }


    public:
        ros::NodeHandle nh_, nh_priv_;
        ros::Subscriber haptic_sub_;
        std::string name_;
        std::string joint_mapping_;
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
