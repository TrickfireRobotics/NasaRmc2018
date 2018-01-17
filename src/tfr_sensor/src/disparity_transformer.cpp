/**
 * Simple wrapper to extend the functionality of the duo3d driver. It publishes
 * a colored disparity /sensor_msgs/Image, and rtabmap needs a
 * stereo_msgs/DisparityImage. 
 *
 * What needs to be done is to wrap the already processed image, supply some
 * calibration information, which has already been calculated by duo.
 *
 * Accepts the namespace for the camera, and expects the published image topic
 * in ./depth/image_raw
 * Publishes on ./depth/image_raw_transformed
 *
 * Subscribed Topics:
 *  -depth/image_raw
 *  -depth/camera_info
 * 
 * Published Topics:
 *  -depth/image_raw_transformed
 *
 * Parameters:
 *  -camera_namespace: set by user
 *  -min_disparity: set by duo
 *  -max_disparity: set by duo
 *  -delta_disparity: set by duo
 *  -focal_length: set by duo
 *  -baseline: set by duo
 * */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <image_transport/image_transport.h>

namespace settings 
{
    const static std::string depth_namespace = "depth/image_raw";
    float baseline;
    float focal_length;
    float min_disparity;
    float max_disparity;
    float delta_disparity;
}

/**
 * Simple wrapper for a ros main to allow for simeltaneous publishing and
 * subscribing
 * */
class Transformer
{

    public:
        Transformer(ros::NodeHandle &nh) : n{nh}
        {
            //initialize message passing
            subscriber = n.subscribe(settings::depth_namespace, 
                    5, &Transformer::transform_data, this);


            publisher = n.advertise<stereo_msgs::DisparityImage>(
                    settings::depth_namespace + "_transformed", 5);


        }
        /**
         * Wraps a standard ros disparity image message around a duo3d disparity
         * image
         * */
        void transform_data(const sensor_msgs::ImageConstPtr &msg )
        {
            stereo_msgs::DisparityImage output{};
            //package up the image
            output.image = *msg;
            output.header = msg->header;

            //parameters from duo calibration
            output.f = settings::focal_length;            //focal length
            output.T = settings::baseline;            //Baseline

            //parameters from duo stereo algorithm
            output.min_disparity = settings::min_disparity;
            output.max_disparity = settings::max_disparity;
            output.delta_d = settings::delta_disparity;

            //construct a basic region of interest, want whole window
            sensor_msgs::RegionOfInterest region;
            region.x_offset = 0; 
            region.y_offset = 0;
            region.height = msg->height;
            region.width = msg->width;
            region.do_rectify = false;

            output.valid_window = region;

            //fire it off!
            //nst image_trans
            publisher.publish(output);
        }    
    private:
        ros::NodeHandle &n;
        ros::Publisher publisher;
        ros::Subscriber subscriber;

        

};

int main(int argc, char **argv)
{
    //node initialization
    ros::init(argc, argv, "disparity_transformer");
    ros::NodeHandle n;
    //get user parameters

    if (!n.getParam("min_disparity", settings::min_disparity))
        settings::min_disparity=0.0;
    ROS_INFO("min_disparity: %f", settings::min_disparity);

    if (!n.getParam("max_disparity", settings::max_disparity))
        settings::max_disparity=0.0;
    ROS_INFO("max_disparity: %f", settings::max_disparity);

    if (!n.getParam("delta_disparity", settings::delta_disparity))
        settings::delta_disparity=0.0;
    ROS_INFO("delta_disparity: %f", settings::delta_disparity);

    if (!n.getParam("focal_length", settings::focal_length))
        settings::focal_length=0.0;
    ROS_INFO("focal_length: %f", settings::focal_length);

    if (!n.getParam("baseline", settings::baseline))
        settings::baseline=0.0;
    ROS_INFO("baseline: %f", settings::baseline);



    ros::spin();
}
