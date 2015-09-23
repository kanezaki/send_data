#include <ros/ros.h>
#include <std_msgs/String.h>

int main( int argc, char** argv ){

  ros::init(argc,argv,"input_person_name");
  if( !ros::master::check() )  return 1;

  ros::NodeHandle _nh;
  ros::Publisher _pub = _nh.advertise<std_msgs::String>("person_name_out", 1);
  std_msgs::String _msg;
  
  while( ros::ok() ){
    std::cout << "Please input person's name.: " ;
    std::getline(std::cin, _msg.data);
    _pub.publish(_msg);
    std::cout << "learning " << _msg.data << std::endl;
  }

  return(0);
}
