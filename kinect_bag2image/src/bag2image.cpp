#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <math.h>

#define foreach_boost BOOST_FOREACH
using namespace std;
string int2str(const int &int_temp)  
{  
    string string_temp;
    stringstream stream;  
    stream<<int_temp;  
    string_temp=stream.str();   
	while(string_temp.size()<6)
	{
	  string_temp = "0" + string_temp;
	}
    return string_temp;
}  
std::string double2string(double d)
{
     std::ostringstream os;
     os.precision(3);	
     os.setf(std::ios::fixed);
     if (os << d)
      return os.str();
     else
      return "invalid conversion";
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"bag2image");
    ros::NodeHandle nh;
    string bag_name = argv[1];   
    string path_name = argv[2]; 
    string rgb_path_name = path_name + "/rgb";
    string depth_path_name = path_name + "/depth";
    string time_name = path_name + "/timestamp.txt";
    ofstream myfile(time_name.c_str(),ios::out|ios::trunc);  
    if(!myfile)
    {
       cout<<"error !";
    }
    int rgb_image_index = 0;
    int depth_image_index = 0;
    int rgb_image_sum = 0;
    int depth_image_sum = 0;
    vector<double> rgb_time, depth_time;
    vector<cv::Mat> rgb_mat,depth_mat,depth_align_mat;

    mkdir(rgb_path_name.c_str() ,S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);  
    mkdir(depth_path_name.c_str() ,S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);  

    cout << "Writing data to " << path_name << endl;
    
    rosbag::Bag *bag_in = new rosbag::Bag(bag_name, rosbag::bagmode::Read);
    rosbag::View *full_view = new rosbag::View;
    full_view->addQuery(*bag_in);
    cv::namedWindow("rgb", CV_WINDOW_NORMAL);
    cv::resizeWindow("rgb",960,540);
    foreach_boost (rosbag::MessageInstance const m, *full_view) 
    {
    	ros::Time const &time = m.getTime();
        string const &topic = m.getTopic();
        if (topic == "/kinect2/hd/image_color_rect") {
          rgb_image_sum++;
          rgb_time.push_back(time.toSec());
          sensor_msgs::ImageConstPtr image_ptr = m.instantiate<sensor_msgs::Image>();
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::BGR8);
          cv::Mat img = cv_ptr -> image;
          rgb_mat.push_back(img);
        } 
        else if(topic == "/kinect2/hd/image_depth_rect") {
          depth_image_sum++;
          depth_time.push_back(time.toSec());
          sensor_msgs::ImageConstPtr image_ptr = m.instantiate<sensor_msgs::Image>();
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::TYPE_16UC1);
          cv::Mat img = cv_ptr -> image;
          depth_mat.push_back(img);
        } 
    }
    for(int i=0;i<rgb_time.size();i++)
    {
        myfile << double2string(rgb_time[i]) << "\r\n";
        vector<double> error;
        for(int j=0;j<depth_time.size();j++)
        {
            error.push_back(abs(rgb_time[i]-depth_time[j]));
        }
        int minPosition = min_element(error.begin(),error.end()) - error.begin();
        cv::imshow("rgb",rgb_mat[i]);
        cv::imwrite(rgb_path_name + "/" + int2str(i) + ".png", rgb_mat[i]);
        cv::imwrite(depth_path_name + "/" + int2str(i) + ".png", depth_mat[minPosition]);
        cv::waitKey(1);
        //cout << minPosition << "   " << error[minPosition] << endl;
    }
    
    bag_in->close();
    bag_in = NULL;
    full_view = NULL;
    myfile.close();
    cout << "done!" << endl;
    return 0;
}