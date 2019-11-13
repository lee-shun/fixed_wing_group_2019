#include <opencv2/opencv.hpp>


using namespace std;

int main ( int argc, char** argv )
{
    std::string path = "/home/nuc-1/CapedVideo/imgs/";
    
    cv::VideoCapture cap(0);
    if (cap.isOpened())
	cout << "camera is opened"<< endl;
    else 
	cout << "camera is not opened" << endl;
    
    cv::Mat frame;
/*
    //设置摄像头分辨率640*480 帧率30fps
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FPS, 30.0);
*/
    //保存视频格式为avi, 编码为MJPG
    cv::VideoWriter writer("/home/nuc-1/CapedVideo/test0.avi",CV_FOURCC('M', 'J', 'P', 'G'),30, cv::Size(640,480),true);
    
    //通过总帧数控制拍摄时间，如果为5s的视频，循环5*30次；
    cv::Mat videoPlay;
    int count = 5 * 30;
    cv::namedWindow("videoplay", cv::WINDOW_NORMAL);
    while (count)
    {
	    cap >> videoPlay;
	    writer << videoPlay;
	    cv::imwrite(path+std::to_string(count)+".png",videoPlay);
	    cv::imshow("videoplay", videoPlay);
	    cv::waitKey(30);
	    count --;
    }
    
    return 0;
}

