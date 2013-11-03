#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <Eigen/Dense>

int n_lasers=0;

std::vector<sensor_msgs::LaserScan> scans;
std::vector<std::string> not_considered_topics;

void callback(const sensor_msgs::LaserScan::ConstPtr& msg, int i){
	scans[i]=*msg;
}

bool contains(std::vector<std::string>& v,const std::string s){
	for(int i=0;i<v.size();i++){
		if(v[i].compare(s)==0) return true;
	}
	return false;
}

void parseCmdLine(int argc,char**argv){
	if(argc>1){
		for(int i=1;i<argc;i++){
			std::string s=argv[i];
			not_considered_topics.push_back(s);
		}
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "multiToSingleLaser");
	ros::NodeHandle n;
	parseCmdLine(argc,argv);

	std::vector<ros::Subscriber> subs;

	ros::master::V_TopicInfo t_info;
	ros::master::getTopics(t_info);

	/****  subscribe laser topics ********/
	int k=0;
	sensor_msgs::LaserScan empty;
	for (int i=0;i<t_info.size();i++){
		std::string t_name=t_info[i].name;
		std::string t_type=t_info[i].datatype;
		if(t_type.compare("sensor_msgs/LaserScan")==0){
			if(!contains(not_considered_topics,t_name)){
				scans.push_back(empty);
				ros::Subscriber sub=n.subscribe<sensor_msgs::LaserScan>(t_name,1,boost::bind(callback,_1,k));
				subs.push_back(sub);
				k++;
			}
		}
	}
	n_lasers=scans.size();
	/*************************************/

	ros::Publisher pub=n.advertise<sensor_msgs::LaserScan>("base_scan", 1);
	
	int fps=1000;
	ros::Rate loop_rate(fps);

	sensor_msgs::LaserScan msg_out;
	msg_out.scan_time=0;
	msg_out.angle_min=0;
	msg_out.angle_max=2*M_PI;

	while(n.ok()){

		ros::spinOnce();
		
		msg_out.ranges.clear();
		float range_min=10000;
		float range_max=0;
		float angle_increment=10000;
		int r_size=0;
		for(int i=0;i<n_lasers;i++){
			if(scans[i].ranges.size()>0){
				float sec=(ros::Time::now()-scans[i].header.stamp).toSec();
				if(sec<2){
					msg_out.header.frame_id=scans[i].header.frame_id;
					if (scans[i].range_min<range_min) range_min=scans[i].range_min;
					if (scans[i].range_max>range_max) range_max=scans[i].range_max;
					if (scans[i].angle_increment<angle_increment) angle_increment=scans[i].angle_increment;
					if (scans[i].ranges.size()>r_size) r_size=scans[i].ranges.size();
				}else{
					scans[i].ranges.clear();
				}
			}
		}
		msg_out.ranges.resize(round(2*M_PI/angle_increment));
		msg_out.range_min=range_min;
		msg_out.range_max=range_max;
		msg_out.angle_increment=angle_increment;
		if(msg_out.ranges.size()>0){
			for(int i=0;i<n_lasers;i++){
				if(scans[i].ranges.size()>0){
					for(int k=0;k<scans[i].ranges.size();k++){
						if(scans[i].ranges[k]>scans[i].range_min&&scans[i].ranges[k]<scans[i].range_max){
							float ang=(scans[i].angle_min+k*scans[i].angle_increment);
							if (ang<0) ang+=2*M_PI;
							int index=round(ang/angle_increment);
							if(msg_out.ranges[index]==0||msg_out.ranges[index]>scans[i].ranges[k]){
								msg_out.ranges[index]=scans[i].ranges[k];
							}
						}
					}
				}
			}
		}
		pub.publish(msg_out);
		loop_rate.sleep();
	}

	return 0;
}

