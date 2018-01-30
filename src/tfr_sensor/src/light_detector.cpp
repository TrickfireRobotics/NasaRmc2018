#include <light_detector.h>

/*
 *  add an image to the moving average, drops old images if no longer needed.
 * */
void LightDetector::add_image(const sensor_msgs::ImageConstPtr& msg)
{
    //manage the moving average
    if (brightness.size() >= threshold)
        brightness.pop_back();

    //convert out of std ros image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,
                sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",
                e.what());
        return;
    }

	//prealllocate loop constants for speed
    int rows = cv_ptr->image.rows;
    int cols = cv_ptr->image.cols * cv_ptr->image.channels();
    int i{},j{};
    ColorStats stats{};
    cv::Vec3f intensity{};

	//optimization from opencv tutorials
    if (cv_ptr->image.isContinuous())
    {
        cols *= rows;
        rows = 1;
    }

    for( i = 0; i < rows; ++i)
    {
        for ( j = 0; j < cols; ++j)
        {
        	intensity = cv_ptr->image.at<cv::Vec3f>(i, j);
        	stats.r_ave += intensity[0]/255.0;
        	stats.g_ave += intensity[1]/255.0;
        	stats.b_ave += intensity[2]/255.0;
		}
    }

    brightness.push_front(stats);
}

/*
 *  Has the light been turned on?  
 * */
bool LightDetector::is_on()
{
    //this algorithm isn't defined for one element
    if (brightness.size() < 2)
        return false;

    ColorStats total;
    int ave_size = brightness.size() -1;

    /*
     *Get an average of the color channels for every element exept for the most
     recent addition to the average.
     * */
    std::for_each(++brightness.begin(), brightness.end(), 
            [&total, ave_size] (const ColorStats &stats)
            {
                total.r_ave += stats.r_ave/ave_size;
                total.r_ave += stats.r_ave/ave_size;
                total.r_ave += stats.r_ave/ave_size;
            });

    auto ave_brightness = [](const ColorStats & stats)
    {
        return (stats.r_ave + stats.g_ave + stats.b_ave)/3.0;
    };

    int total_ave = ave_brightness(total);
    int recent_ave = ave_brightness(brightness.front());
    
    return (recent_ave - total_ave) > threshold;
}
