/* This node does pitch and roll for an obstacle detection sensor. */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

//TODO this can be refactored to use templates
class PointCloudTilter
{
    public:
        PointCloudTilter(ros::NodeHandle& n, const std::string& p_f, const std::string& c_f):
            imu_subscriber{n.subscribe("imu", 10, &PointCloudTilter::storeImu, this)},
            data_subscriber{n.subscribe("points", 10, &PointCloudTilter::tiltData, this)},
            tilt_publisher{n.advertise<sensor_msgs::PointCloud2>("tilted_points", 5)},
            parent_frame{p_f},
            child_frame{c_f},
            br{}
        { }

        void publish_transforms()
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = parent_frame;
            transformStamped.child_frame_id = child_frame;
            if (latest_imu != nullptr)
            {
                auto imu = *latest_imu;
                double pitch, roll;
                // roll (x-axis rotation)
                double sinr = +2.0 * (imu.orientation.w * imu.orientation.x +
                        imu.orientation.y * imu.orientation.z);
                double cosr = +1.0 - 2.0 * (imu.orientation.x *
                        imu.orientation.x + imu.orientation.y *
                        imu.orientation.y);
                roll = atan2(sinr, cosr);

                // pitch (y-axis rotation)
                double sinp = +2.0 * (imu.orientation.w * imu.orientation.y
                        - imu.orientation.z * imu.orientation.x);

                if (fabs(sinp) >= 1)
                    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
                else
                    pitch = asin(sinp);

                tf2::Quaternion q_0;
                q_0.setRPY( -roll, -pitch, 0);
                transformStamped.transform.rotation.w = q_0.getW();
                transformStamped.transform.rotation.x = q_0.getX();
                transformStamped.transform.rotation.y = q_0.getY();
                transformStamped.transform.rotation.z = q_0.getZ();
            }
            else
            {
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = parent_frame;
                transformStamped.child_frame_id = child_frame;
                transformStamped.transform.rotation.w = 1;
            }
            br.sendTransform(transformStamped);
        }

        

    private:

        void tiltData(const sensor_msgs::PointCloud2ConstPtr& cloudPtr)
        {
            auto cloud = *cloudPtr;
            cloud.header.frame_id = child_frame;
            tilt_publisher.publish(cloud);
        }

        ros::Subscriber imu_subscriber;
        ros::Subscriber data_subscriber;
        ros::Publisher tilt_publisher;
        sensor_msgs::ImuConstPtr latest_imu;
        const std::string& parent_frame;
        const std::string& child_frame;
        tf2_ros::TransformBroadcaster br;        


        void storeImu(const sensor_msgs::ImuConstPtr &imu)
        {
            latest_imu = imu;
        }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_tilt") ;
    ros::NodeHandle n;

    std::string parent_frame, child_frame;
    ros::param::param<std::string>("~parent_frame", parent_frame, "");
    ros::param::param<std::string>("~child_frame", child_frame, "");

    PointCloudTilter tilter{n, parent_frame, child_frame};

    ros::Rate rate{10};
    while (ros::ok())
    {
        tilter.publish_transforms();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

