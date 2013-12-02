#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define PI 3.14159265
#define DEG_TO_RAD(x) (x * PI / 180.0)
#define MAX_RANGE 100.0 // meters
#define MIN_ANGLE -144 // degrees
#define MAX_ANGLE 144 // degrees
#define DEG_INCREMENT 2 // degrees

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  
  // Publish generated laserscan data with a queue size of 50
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  // -144 to 144 degrees in 2 deg increments, + 1 to include angle 144
  unsigned int num_readings = (MAX_ANGLE-MIN_ANGLE) / DEG_INCREMENT + 1;
  
  // 25 Hz for sonar
  double sonar_frequency = 25;
  
  double ranges[num_readings];
  double intensities[num_readings];
  
  // Amigobot sonar directions in degrees
  // TODO : Move to cfg file
  int sonar_directions[] = {-144, -90, -44, -12, 
                              12,  44,  90, 144
                           };
  
  // Number of sonar sensors (must match sonar_directions)
  int sonar_count = sizeof(sonar_directions)/sizeof(int);

  int count = 0;
  ros::Rate r(1.0); // 1 Hz
  while(n.ok()){
    // Generate empty data outside of the range for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = MAX_RANGE + 1.0;
      intensities[i] = 0;
    }

    // Generate actual sonar data in correct positions
    for (unsigned int i = 0; i < sonar_count; ++i)
    {
      // Place sonar data into correct position in ranges array
      int index = (sonar_directions[i] - MIN_ANGLE)/2;
      ranges[index] = 1.0;
      intensities[index] = 1.0;
    }


    ros::Time scan_time = ros::Time::now();

    // Populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = DEG_TO_RAD(MIN_ANGLE);
    scan.angle_max = DEG_TO_RAD(MAX_ANGLE);
    scan.angle_increment = DEG_TO_RAD(DEG_INCREMENT);
    scan.time_increment = (1.0 / sonar_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 3.0;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan);
    ++count;
    r.sleep();
  }
}