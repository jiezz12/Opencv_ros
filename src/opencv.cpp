#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <sstream> 
#include <string>

//cmakelists添加环境配置参数find_package(OpenCV REQUIRED)

using namespace cv;
bool image_view;
int video_device;

int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "opencv_ros");  
    ros::NodeHandle nh;  
    image_transport::ImageTransport it(nh);  
    image_transport::Publisher pub = it.advertise("camera/image", 10);  
 
    nh.getParam("image_view", image_view);
    nh.getParam("video_device", video_device);


    VideoCapture cap(video_device);  //dev/video0
    if(!cap.isOpened())   
    {  
        ROS_ERROR("Can not opencv video device\n");  
        return 1;  
    }  
    Mat frame;  //定义opencv形式图像参数
    sensor_msgs::ImagePtr msg;  //定义ros形式图像参数
    namedWindow("video");

    ros::Rate loop_rate(5);  
    while (nh.ok()) 
    {  
        cap >> frame;  //摄像头画面赋给frame
        if(!frame.empty()) //画面是否正常
        {  
            /*对图片二次处理*/


            //opencv转ros
            try{
                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception:%s",e.what());
            }
            if(image_view)
            {
                imshow("video",frame);//显示图像窗口
                waitKey(1);//1ms延时
            }
            pub.publish(msg);  
    	}  
    }
    
    ros::spinOnce();  
    loop_rate.sleep();  
}  