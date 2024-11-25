#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <sstream> 
#include <std_msgs/String.h>
#include <string>

//cmakelists添加环境配置参数find_package(OpenCV REQUIRED)

using namespace cv;
using namespace std;

bool image_view = 1;
int video_device;

Mat frame;  //定义opencv形式图像参数
cv_bridge::CvImage cv_img;
sensor_msgs::Image ros_msg;  //定义ros形式图像参数
sensor_msgs::CameraInfo ci;  //定义相机内参
std_msgs::String ros_str;  //定义相机状态参数
//const string camurl = "file:///home/hsj/.ros/camera_info/head_camera.yaml";
const string camurl = ""; //空的会默认找从 camera_calibration 标定的yaml 文件

std::string  camera_frame_id = "opencv_camera"; //定义相机名称

int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "opencv_ros");  
    ros::NodeHandle nh("~");  

    image_transport::ImageTransport it(nh);  
    camera_info_manager::CameraInfoManager camera_info(nh, "head_camera", camurl);

    image_transport::Publisher image_pub = it.advertise("image", 10);  
    ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info",10);
    ros::Publisher status_pub = nh.advertise<std_msgs::String>("status",10);

    nh.getParam("image_view", image_view);
    nh.getParam("video_device", video_device);

    VideoCapture cap(video_device);  //dev/video0
    if(!cap.isOpened())   
    {  
        ROS_ERROR("Can not opencv video device\n");  
        return 1;  
    }  

    namedWindow("video");

    ros::Rate loop_rate(5);  
    while (ros::ok()) 
    {  
        cap >> frame;  //摄像头画面赋给frame
        if(!cap.read(frame)) {
            ros_str.data = "No frame from camera";
            cv::waitKey();
        }
        else {
            ros_str.data = "live";
        }
        if(!frame.empty()) //画面是否正常
        {  
            /*对图片二次处理*/
	        //circle(frame, Point(50, 50), 45, Scalar(255, 0, 0), -1, 8, 0);        
             //opencv转ros
            try
            {
               // ros_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
               
                cv_img.encoding = sensor_msgs::image_encodings::BGR8;
                cv_img.image = frame;
                cv_img.toImageMsg(ros_msg);
                ros_msg.header.stamp= ros::Time::now();
                ros_msg.header.frame_id = camera_frame_id;
                ci = camera_info.getCameraInfo();
                ci.header = ros_msg.header;
                
                // cv_img.encoding = sensor_msgs::image_encodings::BGR8;
                // cv_img.image = frame;
                // cv_img.toImageMsg(ros_msg);
                // ros_msg.header.stamp= ros::Time::now();
                // ci.header = ros_msg.header;
                // ci.width = ros_msg.width;
                // ci.height = ros_msg.height;
                // ci.distortion_model = "head_camera";
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception:%s",e.what());
            }
            if(image_view)
            {
                imshow("video", frame);
                int c = waitKey(1);
		            if (c == 27) {//key:esc
			        ros::shutdown();
		        }
            }
            image_pub.publish(ros_msg);  
            camera_info_pub.publish(ci);
            status_pub.publish(ros_str);
    	}  
    }
    cap.release();			//释放视频

    ros::spinOnce();  
    loop_rate.sleep();  
}  
