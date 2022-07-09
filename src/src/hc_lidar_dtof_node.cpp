
//#include "base/hclidar.h"
#include "LidarTest.h"
#include <stdio.h>
#include <iostream>
#include <string> 
#include <sstream>
#include <algorithm>
#include <fstream>

#include "base/HcData.h"
#include "base/HcSDK.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <signal.h>
#include <stdio.h>
#include "geometry_msgs/Point32.h"
#include <math.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PointStamped.h>
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"

using namespace std;

#ifndef M_PI
#define M_PI 3.1415926
#endif

std::string  g_strLidarID = "";

void sdkCallBackFunErrorCode(int iErrorCode)
{
    //std::cout << "Main: sdkCallBackFunErrorCode ErrorCode=" << iErrorCode << std::endl;
}

void sdkCallBackFunSecondInfo(tsSDKStatistic sInfo)
{
	/*printf("Main: sdkCallBackFunSecondInfo time=%lld s,points=%d,GrayBytes=%d,FPS=%d,speed=%0.2f,PPS=%d,valid=%d,invalid=%d,ErrorPacket=%d\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS, sInfo.iPacketPerSecond, sInfo.iValid, sInfo.iInvalid
		, sInfo.u64ErrorPacketCount);*/

	std::string strFile = "";
	strFile = "FPS_" + g_strLidarID + ".csv";
	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
	sprintf(buff, "%lld,%d,%d,%lld,%0.2f,%d,%d,%d,%d\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS,  sInfo.iValid, sInfo.iInvalid, sInfo.iPacketPerSecond
		, sInfo.u64ErrorPacketCount);

	outFile.write(buff, strlen(buff));
	outFile.close();

	//printf(buff);
	
}

void sdkCallBackFunPointCloud(LstPointCloud lstG)
{
 
	printf("Main: sdkCallBackFunPointCloud Rx Points=%d\n", lstG.size());

	std::string strFile = "Raw_data.csv";


	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
    for(auto sInfo : lstG)
    {
		memset(buff,0,128);
		sprintf(buff, "%lld,%0.3f,%0.3f,%d,%d,%d,%d,%d\n",
			HCHead::getCurrentTimestampUs(), sInfo.dAngle, sInfo.dAngleRaw, sInfo.u16Dist,sInfo.bValid,sInfo.u16Speed,sInfo.u16Gray,sInfo.bOverRange);

		outFile.write(buff, strlen(buff));

		//printf(buff);

    }
		
	outFile.close();

}

void sdkCallBackFunDistQ2(LstNodeDistQ2 lstG)
{
    std::cout << "Main: sdkCallBackFunDistQ2 Rx Points=" << lstG.size() <<std::endl;
    for(auto sInfo : lstG)
    {
        //std::cout << "Main: Angle=" << sInfo.angle_q6_checkbit/64.0f  << ",Dist=" << sInfo.distance_q2/4 << std::endl;
    }
}

int getPort()
{
	printf("Please select COM:\n");
	int iPort = 3;
    std::cin >> iPort;
	return iPort;
}

int getBaud()
{
	printf("Please select COM baud:\n");
	int iBaud = 115200;
	std::cin >> iBaud;
	return iBaud;
}

std::string getLidarModel()
{
	printf("Please select Lidar model:\n");
	std::string str = "X2M";
	std::cin >> str;
	std::transform(str.begin(), str.end(), str.begin(), ::toupper);
	return str;
}

std::vector<float> split(const std::string &s, char delim) 
{
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}

bool IsInIgnoreArray(std::vector<int>ignore_scan_index, int index)
{
    int i=0;
    for(; i< ignore_scan_index.size()-1; )
    {
        if(index > ignore_scan_index[i] && index < ignore_scan_index[i+1] )
        {
            return true;
        }
        i=i+2;
    }
    return false;
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "hc_lidar_dtof_node");
    //HCLidar& device= HCLidar::getInstance();
    int rtn = 0;

    bool bPollMode = true;
    bool bDistQ2 = false;
    bool bLoop = true;
	std::string strVer = getSDKVersion();
    std::cout << "Main: SDK verion=" << strVer.c_str()<< std::endl;

    //auto funErrorCode = std::bind(sdkCallBackFunErrorCode, std::placeholders::_1);
    //device.setCallBackFunErrorCode(funErrorCode);

    auto funSecondInfo = std::bind(sdkCallBackFunSecondInfo, std::placeholders::_1);
    setSDKCallBackFunSecondInfo(funSecondInfo);

    if(!bPollMode)//call back
    {
        auto funPointCloud = std::bind(sdkCallBackFunPointCloud, std::placeholders::_1);
        setSDKCallBackFunPointCloud(funPointCloud);

        auto funDistQ2 = std::bind(sdkCallBackFunDistQ2, std::placeholders::_1);
        setSDKCallBackFunDistQ2(funDistQ2);
    }

#if 0	
	int iPort = getPort();
	std::string strPort;
#ifdef _WIN32
	strPort = "//./com" + std::to_string(iPort);                     // For windows OS
#else
	strPort = "/dev/ttyUSB" + std::to_string(iPort);              // For Linux OS
#endif


	int iBaud = 230400;//int iBaud = getBaud();

	std::string strLidarModel = "T3A";// std::string strLidarModel = getLidarModel();
	strLidarModel = "T3B";
#endif

	std::string port = "/dev/ttyUSB0";
    int baudrate=230400;
    std::string lidar_model = "T3B";

	std::string frame_id;
	double angle_max,angle_min;
	 bool is_ignore_angle;
    int ignore_index;
    double ignore_angle_rad;
    std::vector<int> ignore_scan_index;
    std::string list;
    std::vector<float> ignore_array;
    double max_range, min_range;
    double frequency;

	ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
	ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/ttyUSB0");
    nh_private.param<int>("baudrate", baudrate, 230400);
    nh_private.param<std::string>("lidar_model", lidar_model, "T3B");
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<double>("angle_max", angle_max , 3.141592);
    nh_private.param<double>("angle_min", angle_min , -3.141592);
    nh_private.param<double>("range_max", max_range , 10.0);
    nh_private.param<double>("range_min", min_range , 0.08);
    nh_private.param<bool>("is_ignore_angle", is_ignore_angle , false);
    nh_private.param<std::string>("ignore_array",list,"");
	int iReadTimeoutms = 2;//
	ros::Rate rate(20);

	//printf("port :%s, lidar_model: %s \n" ,port.c_str(), lidar_model.c_str());
	rtn = hcSDKInitialize(port.c_str(), lidar_model.c_str(), baudrate, iReadTimeoutms, bDistQ2, bLoop, bPollMode);
	//rtn = hcSDKInitialize(strPort.c_str());
    if (rtn != 1)
    {
		hcSDKUnInit();
		printf("Main: Init sdk failed!\n");
		getchar();
		exit(0);
		return 0;
    }

    setSDKLidarPowerOn(true);
    setSDKCircleDataMode();
	
	int iCount = 0;
	uint64_t m_PointTime = 1e9 / 3400; //根据采样率算两个激光点时间间隔
    int count=0;

	if(angle_max < angle_min)
	{
        std::swap(angle_max,angle_min);
    }
    if(is_ignore_angle)
    {
        ignore_array = split(list ,',');
        //判断剔除角度是否成对存在
        if(ignore_array.size()%2){
            ROS_ERROR_STREAM("ignore array is odd need be even");
        }
        if(ignore_array.size() == 0){
            is_ignore_angle = false;
            ROS_WARN("ignore array is 0,and  ,not ignore_angle");
        }
    }

    while (rtn && ros::ok())
    {

        if(bPollMode)
        {
            if(bDistQ2)
            {
                LstNodeDistQ2 lstG;
                getSDKScanData(lstG, false);
				printf( "Main: Poll DistQ2 Rx Points=%d\n" ,lstG.size() );
                //for(auto sInfo : lstG)
                {
					//printf("Main: Angle=%0.2f,Dist=%d\n" ,(double)sInfo.angle_q6_checkbit/64.0f  , sInfo.distance_q2/4 );
                }
            }
            else
            {
                LstPointCloud lstG;
				ros::Time start_time = ros::Time::now();
				if (getSDKRxPointClouds(lstG))
				{
					//sdkCallBackFunPointCloud(lstG);

					if (lstG.size() > 0)
                    {
                        ros::Time  end_time = ros::Time::now();
                        //printf("start_time=%lu, end_time=%lu , diff_offset=%ld \n", start_time.toNSec() , end_time.toNSec() , end_time.toNSec()-start_time.toNSec() );
                        
                        //printf("Main: total Rx Points=%d , value point=%d\n", lstG.size(),count);
                        count=0;
                        
                        reverse(lstG.begin(),lstG.end());
                       
                        sensor_msgs::LaserScan scan_msg;
                        ros::Time start_scan_time;                        
                        start_scan_time.sec = start_time.sec;//微妙除以10的5次方
                        start_scan_time.nsec = start_time.nsec;
                        scan_msg.header.stamp = start_scan_time;
                        scan_msg.header.frame_id = frame_id;
                        scan_msg.angle_min = angle_min; 
                        scan_msg.angle_max = angle_max; 
                        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min)/(lstG.size()- 1);       
                        
                       // uint64_t scan_time = m_PointTime * (lstG.size() - 1);
                        //scan_msg.scan_time =  static_cast<double>(scan_time * 1.0 / 1e9);
                        //scan_msg.time_increment =  scan_msg.scan_time/ (double)(lstG.size() - 1);
                        
          
                        uint64_t time_offset_ns = end_time.toNSec()-start_time.toNSec();
                        //printf("end_time-start_time ns = %ld , diff=%f \n", time_offset_ns , time_offset_ns/1000000000.f);
                        scan_msg.scan_time = static_cast<double>((end_time.toNSec()-start_time.toNSec())/1000000000.f);
                        scan_msg.time_increment =  scan_msg.scan_time/ (double)(lstG.size() - 1);
                        
                        scan_msg.range_min = min_range;
                        scan_msg.range_max = max_range;
                        scan_msg.ranges.resize(lstG.size());
                        scan_msg.intensities.resize(lstG.size());  

                        if(is_ignore_angle)
                        {
                            for(unsigned int i =0 ; i < ignore_array.size();i++)
                            {
                                if(ignore_array[i] < -180 && ignore_array[i] > 180)
                                {
                                    ROS_ERROR_STREAM("ignore array should be -180<=  <=180");
                                }
                                ignore_angle_rad = ignore_array[i]*3.141592 / 180 ;
                                ignore_index = std::ceil((ignore_angle_rad - scan_msg.angle_min)/scan_msg.angle_increment);
                                ignore_scan_index.push_back(ignore_index);
                            }
                        }                     
                        
                        double rad_angel;
                        for(int i=0; i < lstG.size(); i++)
                        {
                            if(lstG[i].dAngle > 180.0)
                            {
                                rad_angel= (360-lstG[i].dAngle)*3.141592 /180.0f;
                            }
                            else
                            {
                                rad_angel=-1*lstG[i].dAngle*3.141592 /180.0f;
                            }
                            int index = std::ceil((rad_angel - scan_msg.angle_min)/scan_msg.angle_increment);
                            
                            //判断该点 是否在忽略的角度范围内，如果是，就置0
                            if(is_ignore_angle && IsInIgnoreArray(ignore_scan_index, index))
                            {
                                if(index >=0 && index < lstG.size())
                                {
                                    scan_msg.ranges[index] = 0.0;
                                }                        
                            }
                            else
                            {
                          
                                if(index >=0 && index < lstG.size()) 
                                {
                                    //如果是无效点，即杂，则置0
                                    if(!lstG[i].bValid)
                                    {
                                         scan_msg.ranges[index] = 0.0;
                                    }
                                    else
                                    {
                                        scan_msg.ranges[index] = (lstG[i].u16Dist)/1000.0f;
                                    }
                                    
                                    scan_msg.intensities[i] = lstG[i].u16Gray;
                                }
                            }
                        }
                        scan_pub.publish(scan_msg);

				}
				else
				{
					int iError = getSDKLastErrCode();
					if (iError != LIDAR_SUCCESS)
					{
						printf( "Main: Poll Rx Points error code=%d\n", iError );
						switch (iError)
						{
						case ERR_SHARK_MOTOR_BLOCKED:
							break;
						case ERR_SHARK_INVALID_POINTS:
							break;
						case ERR_LIDAR_SPEED_LOW:
							break;
						case ERR_LIDAR_SPEED_HIGH:
							break;
						case ERR_DISCONNECTED:
							break;
						case ERR_LIDAR_FPS_INVALID:
							break;
						default:
							break;
						}
					}
				}

				                
            }
        }

		}
		
		rate.sleep();
        ros::spinOnce();
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));
        //std::this_thread::yield();
        //printf("main....\n");
    }


	hcSDKUnInit();
    printf("lidar end....\n");
    return 0;

}
