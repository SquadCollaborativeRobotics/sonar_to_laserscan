#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>

#define PI 3.14159265
#define DEG_TO_RAD(x) (x * PI / 180.0)
#define MAX_RANGE 3.0 // meters
#define MIN_ANGLE -144 // degrees
#define MAX_ANGLE 144 // degrees
#define DEG_INCREMENT 2 // degrees
#define NUM_READINGS ((MAX_ANGLE-MIN_ANGLE) / DEG_INCREMENT + 1)

#define NUMBER_OF_SONAR_SENSORS 8
#define magnitude(x,y) sqrt((x)*(x) + (y)*(y))

// Amigobot sonar directions in degrees
// TODO : Move to cfg file
static int sonar_directions[] = {90, 44, 12, -12, -44, -90, -144, 144};

// Initialize sonar data with default values
double sonar_data[NUMBER_OF_SONAR_SENSORS] = {1, 1, 1, 1, 1, 1, 1, 1};
ros::Time sonar_timestamp;

// -144 to 144 degrees in 2 deg increments, plus 1 to include angle max in array

// 25 Hz for sonar
double sonar_frequency = 25;

double ranges[NUM_READINGS];
double intensities[NUM_READINGS];

ros::Publisher scan_pub;

// Uses global sonar_data to generate laserscan data and publish
// Uses global sonar_timestamp to generate timestamp
void publishLaserFromSonar() {
  // Generate empty data outside of the range for our laser scan
  for(unsigned int i = 0; i < NUM_READINGS; ++i){
    ranges[i] = MAX_RANGE + 1.0;
    intensities[i] = 0;
  }

  // Generate actual sonar data in correct positions
  for (unsigned int i = 0; i < NUMBER_OF_SONAR_SENSORS; ++i)
  {
    // Place sonar data into correct position in ranges array
    int index = (sonar_directions[i] - MIN_ANGLE)/DEG_INCREMENT;
    ranges[index] = sonar_data[i];
    intensities[index] = 1.0;
  }

  // Populate the LaserScan message
  sensor_msgs::LaserScan scan;
  scan.header.stamp = sonar_timestamp;
  scan.header.frame_id = "laser_frame";
  scan.angle_min = DEG_TO_RAD(MIN_ANGLE);
  scan.angle_max = DEG_TO_RAD(MAX_ANGLE);
  scan.angle_increment = DEG_TO_RAD(DEG_INCREMENT);
  scan.time_increment = (1.0 / sonar_frequency) / (NUM_READINGS);
  scan.range_min = 0.0;
  scan.range_max = MAX_RANGE;

  scan.ranges.resize(NUM_READINGS);
  scan.intensities.resize(NUM_READINGS);
  for(unsigned int i = 0; i < NUM_READINGS; ++i){
    scan.ranges[i] = ranges[i];
    scan.intensities[i] = intensities[i];
  }
  scan_pub.publish(scan);
}

// Store most recent sonar data
void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg){
  // Extract distances from depth cloud
  for (unsigned int i = 0; i < NUMBER_OF_SONAR_SENSORS; ++i) {
    sonar_data[i] = magnitude(msg->points[i].x, msg->points[i].y);
  }
  // Get time of scan
  sonar_timestamp = msg->header.stamp;
  
  publishLaserFromSonar();
}


int main(int argc, char** argv){
  // Make sure that the number of values in sonar_direction matches number of sensors
  assert(sizeof(sonar_directions)/sizeof(int) == NUMBER_OF_SONAR_SENSORS);


  ros::init(argc, argv, "laser_scan_publisher");
  ros::NodeHandle n;

  // Publish generated laserscan data with a queue size of 50
  scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
  
  // Subscribe to sonar data with queue size of 100
  ros::Subscriber goal_sub = n.subscribe("amigobot_node/sonar", 100, sonarCallback);

  ros::spin();
}