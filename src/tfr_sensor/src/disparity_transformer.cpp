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
 * */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <image_transport/image_transport.h>

namespace settings 
{
    float baseline=0.03107;
    float focal_length=0.002;
    float min_disparity=0.00;
    float max_disparity=64.0;
    float delta_disparity=0.107;
}

/**
 * Simple wrapper for a ros main to allow for simeltaneous publishing and
 * subscribing
 * */
class Transformer
{

    public:
        Transformer(ros::NodeHandle n, image_transport::ImageTransport &itr) : 
            it{itr}
        {
            //TODO there is something really screwy going on here with
            //namespaces. I don't know what, this really should be a parameter
            //initialize message passing
            subscriber = it.subscribe(
                    "/sensors/duo3d/depth/image_raw", 5, 
                    &Transformer::transform_data, this);


            publisher = n.advertise<stereo_msgs::DisparityImage>(
                    "/sensors/duo3d/depth/image_raw_transformed", 5);


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
        image_transport::ImageTransport &it;
        ros::Publisher publisher;
        image_transport::Subscriber subscriber;

        

};

int main(int argc, char **argv)
{
    //node initialization
    ros::init(argc, argv, "disparity_transformer");
    ros::NodeHandle n;
    //get user parameters


    if (!n.getParam("min_disparity", settings::min_disparity))
        settings::min_disparity=0;

    if (!n.getParam("max_disparity", settings::max_disparity))
        settings::max_disparity=64;

    if (!n.getParam("delta_disparity", settings::delta_disparity))
        settings::delta_disparity=0.107;

    if (!n.getParam("focal_length", settings::focal_length))
        settings::focal_length=0.002;

    if (!n.getParam("baseline", settings::baseline))
        settings::baseline=0.03107;

    image_transport::ImageTransport it(n);
    Transformer transformer(n,it);

    ros::spin();
}
