#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <math.h> 

using namespace std;

// lock
bool restart = false;
bool scan_start = true;
bool odom_start = true;
bool map_start  = false;


int mode = 1;

int targetNum,preTargetNum = 0;
float target_x=-0.5,target_y=-0.5;

//map meta data
float origin_x;
float origin_y;
float resolution;
int width,height;
int size;

float now_x = 0,now_y = 0;
int now_x_pixel,now_y_pixel;
float min_num,minDis;

//sensor_msgs::LaserScan ScanMsg;
//nav_msgs::Odometry OdomMsg;
nav_msgs::OccupancyGrid MapMsg;
nav_msgs::OccupancyGrid Map;
std_msgs::String status;


/*
void scan_callback(const sensor_msgs::LaserScan msg)
{
  ScanMsg = msg;
  scan_start = true;
  
}
*/

/*
void odom_callback(const nav_msgs::Odometry msg)
{
  OdomMsg = msg;
  odom_start = true; 
  
}
*/

// get initial map or update map when map size is changed
void map_callback(const nav_msgs::OccupancyGrid msg)
{
  // get map meta data
  origin_x = msg.info.origin.position.x;
  origin_y = msg.info.origin.position.y;
  resolution = msg.info.resolution; 
  width = msg.info.width;
  height = msg.info.height;
  
  //initialize map
  Map = msg;
  MapMsg = msg;
  map_start = true;
  size = width * height;
}

// update map
void map_update_callback(const map_msgs::OccupancyGridUpdate msg)
{
  if(map_start)
  {
    int origin_num = msg.y * width + msg.x;
    for(int i = 0;i < msg.width;i++)
    {
      for(int j = 0;j < msg.height;j++)
      {
        MapMsg.data[origin_num + j*width +i] = msg.data[j * msg.width + i];   
      }
    }
    cout << "map update"<<endl;
  }
}

// get moving status 
void status_callback(const std_msgs::String msg)
{
  status = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_mapping");
    ros::NodeHandle n;
        
    tf::TransformListener listener;

    // publish target map and temporary goal point
    ros::Publisher pub_goal = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
    ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("/myMap", 1000);
    
    
    // subscribe planner's global costmap and robot's moving status  (not use scan and odom data)
    ros::Subscriber sub_map = n.subscribe("/move_base/global_costmap/costmap", 1000, map_callback);
    ros::Subscriber sub_map_update = n.subscribe("/move_base/global_costmap/costmap_updates", 1000, map_update_callback);
    ros::Subscriber sub_status = n.subscribe("/cmd_vel_mux/active", 10, status_callback);
    //ros::Subscriber sub_scan = n.subscribe("/scan", 1000, scan_callback);
    //ros::Subscriber sub_odom = n.subscribe("/odom", 1000, odom_callback);
    
    ros::Rate loop_rate(5);
 

    
    while (ros::ok())
    {     
        nav_msgs::OccupancyGrid myMap;
        tf::StampedTransform transform;
        
        // get current position 
        try{
          listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
        }
        catch(tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(0.1).sleep();
        }
        now_x = transform.getOrigin().x();
        now_y = transform.getOrigin().y(); 
        now_x_pixel = (now_x-origin_x)/resolution;
        now_y_pixel = (now_y-origin_y)/resolution;

        cout<<"now_x = "<<now_x<<" now_y = "<<now_y<<endl;
        cout<<"pixel_x = "<<now_x_pixel<<" pixel_y = "<<now_y_pixel<<endl;
       
        cout<<"moving status: "<<status.data<<endl;
        
        if(scan_start && odom_start && map_start){
             
            // if we can't arrive target position, then we change target
            if(status.data == "idle" && !restart)
            {        
              restart = true;
              Map.data[targetNum] = 100;
              cout <<"change target"<<endl;
            }else{
              // choose target position
              restart = false;
              cout <<"keep going"<<endl;

              for(int i = 0; i < size; i++)
              {
                // ignore edge
                if((i/width) != 0 && (i/width) != (height-1) && (i%width) != 0 && (i%width) != (width - 1))
                {
                  // past target we couldn't arrive
                  if(Map.data[i] == 100)
                  {
                    continue;
                  }
                  if(MapMsg.data[i] != 0)
                  {
                    Map.data[i] = -1;
                  }
                  else // choose the edge point of found area (pixel value equal to 0) 
                  {
                    if(MapMsg.data[i+1] == -1 || MapMsg.data[i-1] == -1 || MapMsg.data[i+width] == -1 || MapMsg.data[i-width] == -1)
                    {
                      Map.data[i] = 0;
                    }else{
                      Map.data[i] = -1;
                    }
                  }
                }
              }
              
              //choose the closest point
              minDis = 1000000.0;
              for(int i = 0; i < size; i++)
              {
                if(Map.data[i] == 0)
                {
                  int x = i%width;
                  int y = i/width;
                  int dis = pow((x - now_x_pixel),2)+pow((y - now_y_pixel),2);
                  if(dis < minDis)
                  {
                    minDis = dis; 
                    min_num = i;
                  }
                }
              }
              targetNum = min_num;
              
              cout <<"targetNum = "<<targetNum << endl;
              target_x = (targetNum%width+0.5)*resolution + origin_x;
              target_y = (targetNum/width+0.5)*resolution + origin_y;
    
              cout<<"target = <"<<target_x<<","<<target_y<<">"<<endl<<endl;              
              
              // publish target goal
              move_base_msgs::MoveBaseActionGoal msg;
              msg.goal.target_pose.header.frame_id = "map";
              msg.goal.target_pose.pose.position.x = target_x;
              msg.goal.target_pose.pose.position.y = target_y;
              msg.goal.target_pose.pose.orientation.w = 1;
              if(preTargetNum != targetNum)
                pub_goal.publish(msg);
              preTargetNum = targetNum;
           } 
        }else{
          cout<<"not ready"<<endl;
        }
        
        //publish target map
        myMap = Map;
        pub_map.publish(myMap);
 
        ros::spinOnce();
	loop_rate.sleep();
    }
    
    return 0;	
}



