#ifndef INCLUDE_XPLORER_NAVIGATOR_HPP_
#define INCLUDE_XPLORER_NAVIGATOR_HPP_

class navigator {
  private:
    obstacleDetector obsDet;
    ros::NodeHandle n2;
    ros::Publisher pub2;
    geometry_msgs::Twist msg;
  public:
    navigator();
    ~navigator();
    void move();
};

#endif  //INCLUDE_XPLORER_NAVIGATOR_HPP_
