//test code to never be seen again
#include <ros/ros.h>
#include <tfr_utilities/status_publisher.h>
#include <tfr_utilities/status_code.h>
#include <tfr_msgs/EmptySrv.h>


class ToggleTest
{
    public:
        ToggleTest(ros::NodeHandle &n):
            status{},
            motor{n.advertiseService("toggle_motors", &ToggleTest::toggle, this)}
        {}

        bool toggle(tfr_msgs::EmptySrv::Request &request, 
                tfr_msgs::EmptySrv::Response &response)
        {
            status.info(StatusCode::SYS_MOTOR_TOGGLE, 0);
        }

    private:


        StatusPublisher status;
        ros::ServiceServer motor;
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_toggle_test");
    ros::NodeHandle n;
    ToggleTest test{n};
    ros::spin();
    return 0;
}
