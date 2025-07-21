#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <cmath>
#include <opencv2/plot.hpp>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>



using namespace std;
using namespace cv;
using namespace ros;
cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImage image_enhance;
image_transport::Publisher image_pub;
Mat img_brightCorr;
Mat imageHSV,HSV[3];
bool flag = false;//ture为过亮；false为过暗


Mat imgEnlarge(Mat image)
{

    if (image.empty())
    {
        cout<<"读取错误"<<endl;
        
        return image;
    }

    //转换为色彩空间
    cvtColor(image,imageHSV,COLOR_BGR2HSV);
    split(imageHSV,HSV);

    //高斯滤波
    // GaussianBlur(HSV[2],HSV[2],Size(5,5),0);
    
    //计算灰度均值((gray_avg-128)/128) < (-0.3)
    double gray_avg = cv::mean(HSV[2])[0];
    cout<<gray_avg<<endl;

    //判断图像类型
    if (((gray_avg-128)/128) > 0.3)
    {
        //需要进行亮度矫正,并将亮图像取反
        cout<<"过亮需矫正"<<endl;
        img_brightCorr = Scalar::all(255)-HSV[2];
        flag = true;

    }else if ( ((gray_avg-128)/128) < (-0.3))
    {
        /* code */
        cout<<"过暗需矫正"<<endl;
        img_brightCorr = HSV[2];
        flag = false;
    }
    else
    {
        cout<<"此图像无需亮度矫正"<<endl;
        return image;
    }
    //计算取反后的灰度等级直方图
    // 定义直方图参数
    int histSize = 256;  // 直方图的大小
    float range[] = {0, 256};  // 灰度值范围
    const float* histRange = {range};
    bool uniform = true;  // 直方图是否均匀
    bool accumulate = false;  // 直方图是否累加
    Mat hist;
    calcHist(&img_brightCorr,1,nullptr,Mat(),hist,1,&histSize,&histRange,uniform,accumulate);
    
    double pMin,pMax;//直方图的最值
    minMaxLoc(hist,&pMin,&pMax);

    //计算由加权分布函数平滑后的直方图
    Mat pwHist = hist.clone() ;//平滑后的直方图，初始化
    //权重因子
    float alph;
    if (flag)
    {
        alph = 0.25;
    }else
    {
        alph = 0.75;
    }   
    for (int i = 0; i < 256; i++)
    {
        float temp = ((hist.at<float>(i) - pMin)/(pMax-pMin));
        pwHist.at<float>(i) = pMax * pow(temp,alph);
    }
    
    //绝对值求和归一化
    Mat pwHist_;
    normalize(pwHist,pwHist_,1,0,NORM_L1,-1,noArray());

    //计算图像的加权分布函数Cw（l),计算亮度矫正伽马
    Mat  Cw = pwHist_.clone();
    Mat  gammaR = Cw.clone();
    float tao = 0.50;
    for (int i = 0; i < 256; i++)
    {
        float temp = 0.0;
        for (int j = 0; j <= i; j++)
        {
            temp += pwHist_.at<float>(j);
        }
        Cw.at<float>(i) = temp;
        if (flag)
        {
            
            gammaR.at<float>(i) = 1-Cw.at<float>(i);

        }else
        {
            gammaR.at<float>(i) = std::max(tao,(1-Cw.at<float>(i)));
        }   
    }

    //进行像素值变换
    Mat  img_brightCorr_ = img_brightCorr.clone();
    for (int i = 0; i < img_brightCorr_.rows; i++)
    {
        for (int j = 0; j < img_brightCorr_.cols; j++)
        {
            //得到当前像素的灰度值
            int temp = static_cast<int>(img_brightCorr_.at<uchar>(i,j));

            //计算变换后的灰度值
            int gray_new =  round(255 * pow((temp/255.0),gammaR.at<float>(temp)));
            img_brightCorr_.at<uchar>(i,j) = static_cast<uchar>(gray_new);
        }
        
    }

    if (flag)
    {    
        img_brightCorr_ = Scalar::all(255)-img_brightCorr_;
    }
        
    //亮度矫正并转换到颜色空间
    vector<Mat>channels = {HSV[0],HSV[1],img_brightCorr_};
    merge(channels,image);
    cvtColor(image,image,COLOR_HSV2BGR);
    return image;
}

void imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cout<<"执行回调函数"<<endl;
    try
        {
           cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception & e)
        {
            std::cerr << e.what() << '\n';
            return;
        }
    image_enhance.header = msg->header;
    image_enhance.encoding = sensor_msgs::image_encodings::BGR8;
    image_enhance.image = imgEnlarge(cv_ptr->image);

    image_pub.publish(image_enhance.toImageMsg());
}
int main(int argc, char  **argv)
{   
    init(argc,argv,"image_enhance");
    NodeHandle nh;
    image_transport::ImageTransport it(nh);
    //订阅图像
   image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw",1,imageCb);

    image_pub = it.advertise("/processed_image", 1);

    spin();

    return 0;
}