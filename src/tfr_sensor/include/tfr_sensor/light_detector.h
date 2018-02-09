#ifndef LIGHT_DETECTOR_H
#define LIGHT_DETECTOR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <deque>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/**
 *  A utility object to detect if a light has been turned on in a stream of
 *  images
 *
 *  Keeps a moving average of image brightness and detects spikes within a fast
 *  window, designed to be used with streaming data. 
 * */
class LightDetector
{
    public:
        
        /*
         * width - how many images to incorperate into the average
         * threshold - as a ratio, how much or a brightness increase is
         * considered turning the light on.
         * */
        LightDetector(int w, double t): 
            brightness{}, width{w}, threshold{t} {};

        ~LightDetector() = default;
        LightDetector(const LightDetector&) = delete;
        LightDetector& operator=(const LightDetector&) = delete;
        LightDetector(LightDetector&&) = delete;
        LightDetector& operator=(LightDetector&&) = delete;
        
        //add an image to average
        void add_image(const sensor_msgs::ImageConstPtr& msg);
        //has the light been turned on?
        bool is_on();
        void clear();
    private:

        struct ColorStats { double r_ave, g_ave, b_ave; };

        std::deque<ColorStats> brightness;
        int width;
        double threshold;
};
#endif
