#ifndef INCLUDE_XPLORER_OBSTACLEDETECTOR_HPP_
#define INCLUDE_XPLORER_OBSTACLEDETECTOR_HPP_

class obstacleDetector {
  private:
    ros::NodeHandle n1;
    ros::Subscriber sub1;
    ros::Publisher pub1;
    bool collision;
  public:
    obstacleDetector();
    ~obstacleDetector();
    void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

#endif //INCLUDE_XPLORER_OBSTACLEDETECTOR_HPP_


