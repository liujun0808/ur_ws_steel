#include "ur10_start/my_vision_manager.h"


my_vision_manager::my_vision_manager(ros::NodeHandle n_):it_(n_)
{
    // 矩阵初始化
    //对齐后的内参矩阵
    this->camera_k<<607.35791015625, 0.0, 313.58734130859375, 0.0, 607.50048828125, 251.40821838378906, 0.0, 0.0, 1.0;

    Eigen::Quaterniond camera_end_Q(0.9998009506716774,-0.002700304525803343,-0.01706947995176722,-0.009969967184214884);//相机到末端的四元数
    this->camera_end_rMat=camera_end_Q.normalized().toRotationMatrix();
    //外参矩阵
    this->camera2tool0<<camera_end_rMat(0,0), camera_end_rMat(0,1) , camera_end_rMat(0,2), -0.011147590527141926,
                    camera_end_rMat(1,0), camera_end_rMat(1,1),  camera_end_rMat(1,2), -0.10696187899714274,
                    camera_end_rMat(2,0), camera_end_rMat(2,1),  camera_end_rMat(2,2), -0.13623781079643585,
                    0.0, 0.0, 0.0, 1.0;
    // std::cout<<"外参矩阵：\n"<<camera2tool0<<std::endl;
    n_.subscribe("objects",1,&my_vision_manager::objectsDetectedCb,this);
    // image_color = it_.subscribe("camera/color/image_raw",1,&my_vision_manager::imageCb,this);
    // image_color = it_.subscribe("objects",1,&my_vision_manager::objectsDetectedCb,this);
    image_depth = it_.subscribe("camera/aligned_depth_to_color/image_raw",1,&my_vision_manager::imageCb_d,this);
    ROS_INFO("构造函数");
}



void my_vision_manager::detect2DObject(cv::Mat msg,float &x,float &y)
{
    cv::Mat BGR[3];

    ROS_INFO("detect3Dlocation");

    cv::Mat image =msg.clone();

    cv::split(image,BGR);
    cv::Mat gray_image_red = BGR[2];
    cv::Mat binareyImage;
    cv::medianBlur(gray_image_red,binareyImage,3);

    // 二值分割
    for (int i = 0; i < binareyImage.rows; i++)
    {
        for (int j = 0; j < binareyImage.cols; j++)
        {
            int pixel_value = binareyImage.at<uchar>(i,j);
            if ((pixel_value >= 253) && (pixel_value <= 255))
            {
                binareyImage.at<uchar>(i,j) = 255;
            }
            else
            {
                 binareyImage.at<uchar>(i,j) = 0;
            }
            
        }
        
    }
    cv::dilate(binareyImage,binareyImage,cv::Mat());

    //找非0点，计算目标中心点
    std::vector<cv::Point> nonZeroPoints;
    cv::findNonZero(binareyImage,nonZeroPoints);
    cv::Rect rect = cv::boundingRect(nonZeroPoints);
    //目标像素中心值
    x = rect.x + rect.width/2;
    y = rect.y + rect.height/2;

    // 对齐至深度图的像素坐标
    curr_pixel_centre_x = x;
    curr_pixel_centre_y = y;
    // Eigen::MatrixXi pixel2color(3, 1);
    // pixel2color<<x,y,0;
    // Eigen::MatrixXi pixel2depth(3, 1);
    // pixel2depth = color2depth * pixel2color;
    // curr_pixel_centre_x = pixel2color(0,0);
    // curr_pixel_centre_y = pixel2color(1,0);

    //画轮廓
    cv::Point pt;
    pt.x = x;
    pt.y = y;
    cv::circle(image, pt, 4, cv::Scalar(0, 255, 0), -1, 8);//在中心点画圆

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binareyImage,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,cv::Point(0,0));

    for (int i = 0; i < contours.size(); i++)
    {
        cv::drawContours(image,contours,i,cv::Scalar(255,0,0),2,8,hierarchy,0,cv::Point(0,0));
    }
    cv::namedWindow("Centre point", cv::WINDOW_AUTOSIZE);
	cv::imshow("Centre point", image);
	cv::waitKey(100);   
}


void my_vision_manager::convertToMM()
{   
    //矩阵初始化
    pixel_location = Eigen::MatrixXd::Identity(3, 1);   //用单位矩阵初始化
    camera_location = Eigen::MatrixXd::Identity(3, 1);   //用单位矩阵初始化
    point2cam_4 = Eigen::MatrixXd::Identity(4, 1);
    point2tool_4 = Eigen::MatrixXd::Identity(4, 1);
    Eigen::MatrixXd point2tool_4_inv;
    point2tool_4_inv = Eigen::MatrixXd::Identity(4, 1);

    //像素值改写为3*1矩阵
    pixel_location << center.x * 1.0, center.y * 1.0, 1.0;
    std::cout <<"点在像素坐标系下 "<<pixel_location.transpose()<<std::endl;

    //相机坐标系下的平面坐标=内参矩阵的逆*像素坐标
    camera_location = camera_k.inverse().eval()* pixel_location; 

    // 场景实际三维坐标(深度相机坐标系下的抓取点三维坐标) = 相机的3维坐标 x 深度相机的抓取点深度
    camera_location(0, 0) = camera_location(0, 0) * curr_pixel_centre_z*0.001;
    camera_location(1, 0) = camera_location(1, 0) * curr_pixel_centre_z*0.001;
    camera_location(2, 0) = curr_pixel_centre_z*0.001;

    //坐标齐次化
    point2cam_4<<camera_location(0,0),camera_location(1,0),camera_location(2,0),1;
    std::cout <<"点在相机坐标系下: "<<point2cam_4.transpose()<<std::endl;

    //点到末端tool0坐标系下的位置
    point2tool_4 = camera2tool0 * point2cam_4;
    std::cout <<"点在末端标系下: "<<point2tool_4.transpose() <<std::endl;

    //点到末端tool0坐标系下的位置inv
    point2tool_4_inv= camera2tool0.inverse().eval() * point2cam_4;
    std::cout <<"求逆后点在末端标系下: "<<point2tool_4_inv.transpose() <<std::endl;

}
void my_vision_manager::grasp()
{
    
}

void my_vision_manager::imageCb(const sensor_msgs::ImageConstPtr &msg)
{

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception & e)
    {
        std::cerr << e.what() << '\n';
    }
    
    ROS_INFO("执行回调函数imageCb");
    float obj_x,obj_y;

    detect2DObject(cv_ptr->image,obj_x,obj_y);

    ROS_INFO("目标的像素值:%0.5f  %0.5f",obj_x,obj_y);

}
void my_vision_manager::imageCb_d(const sensor_msgs::ImageConstPtr &msg_d)
{
    try
    {
        cv_ptr_d = cv_bridge::toCvCopy(msg_d,sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch(cv_bridge::Exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    ROS_INFO("执行回调函数imageCb_d");
    // cv::Mat depth_image = cv_ptr_d->image;

    // // 1. 应用滤波器（示例：中值滤波）
    // cv::medianBlur(depth_image, depth_image, 5);

    // // 2. 填充无效区域（示例：使用最近有效像素填充）
    // cv::Mat valid_mask = (depth_image > 0);
    // cv::Mat filled_depth_image;
    // cv::inpaint(depth_image, valid_mask, filled_depth_image, 3, cv::INPAINT_NS);

    // // 3. 平滑深度图像（示例：高斯平滑）
    // cv::GaussianBlur(filled_depth_image, filled_depth_image, cv::Size(5, 5), 0);


    curr_pixel_centre_z = cv_ptr_d->image.at<float>(int(curr_pixel_centre_y),int(curr_pixel_centre_x));
    // detectDepth(cv_ptr_d->image);
    ROS_INFO("%f",curr_pixel_centre_z);
    convertToMM();
}

void my_vision_manager::objectsDetectedCb(const std_msgs::Float32MultiArrayConstPtr & msg)
{
    printf("---\n");
    const std::vector<float> & data = msg->data;
	if(data.size())
	{
		for(unsigned int i=0; i<data.size(); i+=12)
		{
			// get data
			int id = (int)data[i];
			float objectWidth = data[i+1];
			float objectHeight = data[i+2];

			// Find corners Qt
		// 	QTransform qtHomography(data[i+3], data[i+4], data[i+5],
		// 							data[i+6], data[i+7], data[i+8],
		// 							data[i+9], data[i+10], data[i+11]);

		// 	QPointF qtTopLeft = qtHomography.map(QPointF(0,0));
		// 	QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0));
		// 	QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight));
		// 	QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight));

		// 	printf("Object %d detected, Qt corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
		// 			id,
		// 			qtTopLeft.x(), qtTopLeft.y(),
		// 			qtTopRight.x(), qtTopRight.y(),
		// 			qtBottomLeft.x(), qtBottomLeft.y(),
		// 			qtBottomRight.x(), qtBottomRight.y());
        // double x = (qtTopRight.x() - qtTopLeft.x())/2+qtTopLeft.x();
        // x = int(x);
        // double y = (qtBottomLeft.y() - qtTopLeft.y())/2+qtTopLeft.y();
        // y = int(y);
		}
	}
	else
	{
		printf("No objects detected.\n");
	}
    std::cout<<"中心点像素：\n"<<center<<std::endl;
    
}

int main(int argc, char  **argv)
{   
        // vm.convertToMM(point2Cam);
    setlocale(LC_ALL,"");//打印中文
    ros::init(argc,argv,"my_vision_manager");
    ros::NodeHandle n_; 
    ROS_INFO("wait 2s....");
    ros::WallDuration(2).sleep();
    Eigen::Matrix<double,4,1> point2Cam;
    my_vision_manager vm(n_);

    while (ros::ok())
    {
        ros::spinOnce();
        ros::WallDuration(1).sleep();
    }
    // ros::shutdown();
    return 0;
}
