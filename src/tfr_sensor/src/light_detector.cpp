#include <light_detector.h>

/*
 *  add an image to the moving average, drops old images if no longer needed.
 * */
void LightDetector::add_image(const sensor_msgs::ImageConstPtr& msg)
{
    //manage the moving average
    if (brightness.size() >= width)
        brightness.pop_back();
    
    //convert out of std ros image
    cv::Mat image;
    try
    {
        image = cv_bridge::toCvCopy(msg)->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",
                e.what());
        return;
    }
    cv::Scalar intensities = cv::sum(image);

    ColorStats stats{};

    //note we have to reverse out of native cv bgr ordering
    stats.r_ave = intensities[2]/(image.rows*image.cols);
    stats.g_ave = intensities[1]/(image.rows*image.cols);
    stats.b_ave = intensities[0]/(image.rows*image.cols);

    brightness.push_front(stats);
}

/*
 *  Has the light been turned on?  
 * */
bool LightDetector::is_on()
{
    //we want a full window this will take 1/20th of a second to fill
    if (brightness.size() < width)
        return false;

    ColorStats total{};
    int ave_size = brightness.size() -  1;

    /*
     * Get an average of the color channels for every element exept for the most
     * recent addition to the average.
     * */
    for (auto current = ++brightness.begin(), end = brightness.end(); 
            current != end; current++)
    {
        total.r_ave += current->r_ave/ave_size;
        total.g_ave += current->g_ave/ave_size;
        total.b_ave += current->b_ave/ave_size;
    }

    //ROS_INFO("int: %f %f %f", total.r_ave, total.g_ave, total.b_ave);
    ColorStats recent = brightness.front();

    double delta_b = recent.b_ave - total.b_ave; 
    double delta_g = recent.g_ave - total.g_ave; 
    double delta_r = recent.r_ave - total.r_ave; 
    double ave_delta = (delta_g + delta_r)/2;


    ROS_INFO("%f %f %f %f %f", delta_r, delta_g, delta_b, ave_delta,
            delta_b-ave_delta);


    //lets keep this for when we tune it to a color
    return (delta_b-ave_delta) > threshold*ave_delta;
}

/*
 * clear out the average
 */
void LightDetector::clear()
{
    brightness.clear();
}
