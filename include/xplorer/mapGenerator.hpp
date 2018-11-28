#ifndef INCLUDE_XPLORER_MAPGENERATOR_HPP_
#define INCLUDE_XPLORER_MAPGENERATOR_HPP_

class mapGenerator {
  private:
    navigator nav;
    ros::NodeHandle n3;
    ros::Publisher pub3;
    octomap_msgs::Octomap mapMsg;
  public:
    mapGenerator();
    ~mapGenerator();
    void createMap();
};

#endif  //INCLUDE_XPLORER_MAPGENERATOR_HPP_
