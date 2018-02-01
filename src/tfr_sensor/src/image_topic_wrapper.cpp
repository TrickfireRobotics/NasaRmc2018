/**
 * wrapper for an image stream, allows the user to get the most recent image
 * from that stream on demand through a service. 
 *
 * The names of the service and sensor stream are configurable by the user.
 *
 * Subscribed Topics:
 * <camera_topic>: user suppplied
 * Provided Services:
 * <service_name>: user supplied
 *
 * Parameters:
 * ~camera_topic: the camera topic to subscribe to (string, default: "")
 * ~service_name: the name of the service to advertise (string, default: "")
 * 
 * Relevant Messages:
 * tfr_msgs::WrappedImage (srv)
 * */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <tfr_msgs/WrappedImage.h>

class ImageWrapper
{
    public:

        ImageWrapper(ros::NodeHandle &n, const std::string &camera_topic,
                const std::string &service_name)
        {
            image_transport::ImageTransport it(n);
            subscriber = it.subscribe(camera_topic, 20, &ImageWrapper::set_current, this);
            server = n.advertiseService(service_name, &ImageWrapper::get_current, this);
        }
        
        ~ImageWrapper() = default;
        ImageWrapper(const ImageWrapper&) = delete;
        ImageWrapper& operator=(const ImageWrapper&) = delete;
        ImageWrapper(ImageWrapper&&) = delete;
        ImageWrapper& operator=(ImageWrapper&&) = delete;

    private:

        //subscription callback
        void set_current(const sensor_msgs::ImageConstPtr &image)
        {
            //this is safe because of smart pointers and non threaded spinning
            current = *image;
        }

        //service callback
        bool get_current(tfr_msgs::WrappedImage::Request &request,
                tfr_msgs::WrappedImage::Response &response)
        {
            response.image = current;
            return true;
        }
        
        image_transport::Subscriber subscriber;
        ros::ServiceServer server;
        sensor_msgs::Image current{};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_topic_wrapper");
    ros::NodeHandle n;
    std::string camera_topic{}, service_name{};
    ros::param::param<std::string>("~camera_topic", camera_topic, "");
    ros::param::param<std::string>("~service_name", service_name, "");
    ImageWrapper wrapper{n, camera_topic, service_name};
    ros::spin();
}
