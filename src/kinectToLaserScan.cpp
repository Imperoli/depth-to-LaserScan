#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <Eigen/Dense>


//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"

using namespace Eigen;

int width=0;
int height=0;
float fov_x=0;
Eigen::MatrixXf depth_im;
Eigen::Matrix<float, 3,3> K;
Eigen::Matrix<float, 3,3> invK;
Matrix< float, 4, 4 > T;

std::vector<float> data_out;
float range_min=0.1;
float range_max=100;
float scan_time=0;
float angle_min;
float angle_max;
float angle_increment=0;

std::string source_frame="";
std::string target_frame="";

ros::Time time_stamp;

//cv::Mat im(240,320,CV_8UC3);

void cameraInfo_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			K(j,i)=msg->K[i+j*3];
		}
	}
	K(2,2)=1.f;
	invK=K.inverse();
	fov_x = atan( (K(0,2)) / K(0,0) )*2;
	angle_min=0;
	angle_max=2*M_PI;
	if (width!=0) angle_increment=fov_x/width;
}

void cameraData_callback(const sensor_msgs::Image::ConstPtr& msg){
	width=msg->width;
	height=msg->height;
	depth_im.resize(height,width);

	//im=cv::Mat(height,width,CV_8UC3);

	time_stamp=ros::Time::now();
	
	// fill depth_im//
	const unsigned char *c_data = &(msg->data[0]);
	const float *data = reinterpret_cast<const float *>(c_data);
	for(int i=0;i<height;i++){
		for(int j=0;j<width;j++){
			depth_im(i,j)=data[j+i*width];
		}
	}

	//for depth image visualization
	/*for(int i=0;i<height;i++){
		for(int j=0;j<width;j++){
			//im.at<float>(i,width-1-j)=depth_im(i,j)/max;
			im.at<cv::Vec3b>(i,j)=cv::Vec3b(round(255*depth_im(i,j)/3),round(255*depth_im(i,j)/3),round(255*depth_im(i,j)/3));
		}
	}*/
}


void depthImageToLaserScanData(){
	//data_out.clear();
	data_out.assign((int)round(2*M_PI/angle_increment),-1);
	float max_offset=0.2;
	Eigen::Vector3f v;
	Eigen::Vector4f v2;

	/*for(int i=0;i<height;i++){
		for(int j=0;j<width;j++){
			//im.at<float>(i,width-1-j)=depth_im(i,j)/max;
			im.at<cv::Vec3b>(i,j)=cv::Vec3b(round(255*depth_im(i,j)/3),round(255*depth_im(i,j)/3),round(255*depth_im(i,j)/3));
		}
	}*/
	
	for (int j=0;j<width;j++){
		for(int i=0;i<height;i++){
			if(depth_im(i,j)==depth_im(i,j)){
				v=depth_im(i,j)*invK*Eigen::Vector3f(j,i,1);
				v2=T*Eigen::Vector4f(v(2),-v(0),-v(1),1);
				v=v2.head(3)/v2(3);
				if(v(2)<max_offset&&v(2)>-max_offset){
					float dist=v.norm();
					if(dist<range_max){
						float dist2=Eigen::Vector2f(v(0),v(1)).norm();
						float angle=(atan2(v(0),-v(1))-M_PI/2);
						
						if (angle<0){
							angle+=2*M_PI;
						}
						float index=angle/angle_increment;
						//im.at<cv::Vec3b>(i,j)=cv::Vec3b(round(100*depth_im(i,j)/3),round(100*depth_im(i,j)/3),round(220*depth_im(i,j)/3));
						if (data_out[(int)round(index)]<0||data_out[(int)round(index)]>dist2){
							data_out[(int)round(index)]=dist2;
						}
					}
				}
			}
		}
	}
}


void parseCmdLine(int argc, char** argv){
	if(argc==3){
		source_frame=argv[2];
		target_frame=argv[1];
		//std::cout<<source_frame<<" "<<target_frame<<std::endl;
	}
}

int main(int argc, char** argv){

	ros::init (argc, argv, "KinectToLaserScan",ros::init_options::AnonymousName);
	ros::NodeHandle n;

	parseCmdLine(argc, argv);

	ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("laser_0", 1);
	ros::Subscriber sub = n.subscribe ("camera/depth/image", 1, cameraData_callback);
	ros::Subscriber sub2 = n.subscribe ("camera/depth/camera_info", 1, cameraInfo_callback);

	int fps=1000;
	ros::Rate loop_rate(fps);

	sensor_msgs::LaserScan msg_out;
	msg_out.range_min=range_min;
	msg_out.range_max=range_max;
	msg_out.scan_time=scan_time;

	tf::TransformListener listener;
	tf::StampedTransform transform;

	T.setIdentity();

	while(n.ok()){
		ros::spinOnce();
		if(height*width>0&&angle_increment>0){

			try{
				 ros::Time now = ros::Time::now();
    				listener.waitForTransform(target_frame, source_frame,now, ros::Duration(.03));
				listener.lookupTransform(target_frame, source_frame, now,transform);
				
				T.setIdentity();
				tf::Vector3 v;
				tf::Matrix3x3 R=transform.getBasis();
				for(int i=0;i<3;i++){
					v=R[i];
					T.row(i).head(3)=Eigen::Vector3f(v[0],v[1],v[2]);
				}
				v=transform.getOrigin();
				T.col(3).head(3)=Eigen::Vector3f(v[0],v[1],v[2]);

			}
			catch (tf::TransformException ex){
			      ROS_ERROR("%s",ex.what());		
		    	}

			depthImageToLaserScanData();

			
			msg_out.header.stamp=time_stamp;
			msg_out.header.frame_id=target_frame;
			msg_out.angle_min=angle_min;
			msg_out.angle_max=angle_max;
			msg_out.angle_increment=angle_increment;
			msg_out.ranges=data_out;

			pub.publish(msg_out);

			//cv::imshow("depth",im);
			//cv::waitKey(1000/fps);

			loop_rate.sleep();
		}
	}

	return(0);
}




