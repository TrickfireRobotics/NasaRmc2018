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

    double recent_ave = (recent.r_ave + recent.g_ave + recent.b_ave)/3;
    double total_ave = (total.r_ave + total.g_ave + total.b_ave)/3;


    //lets keep this for when we tune it to a color
    ROS_INFO("frame_calculated, recent- total: %f, threshold: %f", recent_ave - total_ave , threshold*total_ave);
    return (recent_ave - total_ave) > threshold*total_ave;
}

/*
 * clear out the average
 */
void LightDetector::clear()
{
    brightness.clear();
}
