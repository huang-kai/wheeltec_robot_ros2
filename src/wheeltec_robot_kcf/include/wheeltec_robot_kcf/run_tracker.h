#ifndef TRANSBOT_ASTRA_RUN_TRACKER_H
#define TRANSBOT_ASTRA_RUN_TRACKER_H

#include <iostream>
#include <algorithm>
#include <dirent.h>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include "kcftracker.h"
#include "PID.h"
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/bool.hpp"
#include <time.h>

using namespace std;
using namespace cv;
using std::placeholders::_1;

class ImageConverter :public rclcpp::Node{
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    
public:
    ImageConverter():Node("image_converter")
    {
    float linear_KP=3.0;
    float linear_KI=0.0;
    float linear_KD=1.0;
    float angular_KP=0.5;
    float angular_KI=0.0;
    float angular_KD=2.0;
    float minDist = 1.0;
    bool refresh = false;
        
    this->declare_parameter<float>("linear_KP_",3.0);
    this->declare_parameter<float>("linear_KI_",0.0);
    this->declare_parameter<float>("linear_KD_",1.0);
    this->declare_parameter<float>("angular_KP_",0.5);
    this->declare_parameter<float>("angular_KI_",0.0);
    this->declare_parameter<float>("angular_KD_",2.0);
    this->declare_parameter<float>("minDist_",1.0);
    this->declare_parameter<bool>("refresh_",false);
     
        
    this->get_parameter<float>("linear_KP_",linear_KP);
    this->get_parameter<float>("linear_KI_",linear_KI);
    this->get_parameter<float>("linear_KD_",linear_KD);
    this->get_parameter<float>("angular_KP_",angular_KP);
    this->get_parameter<float>("angular_KI_",angular_KI);
    this->get_parameter<float>("angular_KD_",angular_KD);
    this->get_parameter<float>("minDist_",minDist);
    this->get_parameter<bool>("refresh_",refresh);

        
    this->linear_PID = new PID(linear_KP, linear_KI, linear_KD);
    this->angular_PID = new PID(angular_KP, angular_KI, angular_KD);
        image_sub_=this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw",1,std::bind(&ImageConverter::imageCb,this,_1));
        depth_sub_=this->create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_raw",1,std::bind(&ImageConverter::depthCb,this,_1));

        image_pub_=this->create_publisher<sensor_msgs::msg::Image>("/KCF_image",1);
        vel_pub_ =this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
    }

    PID *linear_PID;
    PID *angular_PID;

    const char *RGB_WINDOW = "rgb_img";
    const char *DEPTH_WINDOW = "depth_img";
    float minDist = 1.0;
    float linear_speed = 0;
    float rotation_speed = 0;
    bool enable_get_depth = false;
    float dist_val[5];
    bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool LAB = false;
    int center_x;
    KCFTracker tracker;
    
    void PIDcallback();

    void Reset();

    void Cancel();

    void imageCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) ;

    void depthCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) ;

    void JoyCb(const std::shared_ptr<std_msgs::msg::Bool> msg) ;

};


#endif //TRANSBOT_ASTRA_RUN_TRACKER_H
