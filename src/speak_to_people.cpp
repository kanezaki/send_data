#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

// #define DEMO

class Speak {
private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  ros::Publisher _pub;
public:
  Speak() {
    _pub = _nh.advertise<std_msgs::String>("person_name_out", 1);
    _sub = _nh.subscribe ("/person_name_in", 1,  &Speak::name_cb, this);
    ROS_INFO ("Listening for incoming data on topic /person_name_in ..." );
  }

  ~Speak() { }

  //* get name
  void name_cb( const std_msgs::StringConstPtr& msg ){
    if( msg->data.compare("NOBODY")==0 )
      return;

    char cmd[ 1000 ];

#ifdef DEMO
    if( msg->data.compare("SOMEBODY")==0 ){
      // dummy
      std_msgs::String _msg;
      _msg.data = "Sean";

      system( "espeak --stdout \"Hi.\" | aplay" );
      usleep( 500000 );
      system( "espeak --stdout \"Have we met before?\" | aplay" );
      usleep( 2000000 );
      system( "espeak --stdout \"So, what is your name?\" | aplay" );
      usleep( 3000000 );

      _pub.publish(_msg);

      sprintf( cmd, "espeak --stdout \"Nice to meet you, %s. I'm turtlebot.\" | aplay",_msg.data.c_str() );
      system(cmd);
      //usleep( 500000 );
      system( "espeak --stdout \"See you. Have a nice day.\" | aplay" );

      //usleep( 3000000 );
      exit(1); // stop the program.
      return;
    }

    std::cout << "I got a person's name! " << msg->data << std::endl;
    sprintf( cmd, "espeak --stdout \"Hi, %s. How are you?\" | aplay",msg->data.c_str() );
    system(cmd);
    usleep( 1000000 );
    system( "espeak --stdout \"Good. Nice to meet you again. Have a nice day.\" | aplay" );
    usleep( 3000000 ); // wait and continue.
#else
    if( msg->data.compare("SOMEBODY")==0 ){
      system( "espeak --stdout \"Hello. Welcome to Microsoft.\" | aplay" );
    }
    else{
      std::cout << "I got a person's name! " << msg->data << std::endl;
      sprintf( cmd, "espeak --stdout \"Hi, %s. Nice to meet you.\" | aplay",msg->data.c_str() );
      system(cmd);
      //system( "espeak --stdout \"Hello. Nice to meet you again.\" | aplay" );
    }
    system( "espeak --stdout \"Have a nice day.\" | aplay" );
    usleep( 3000000 ); // wait and continue.
#endif
  }
};

int main( int argc, char** argv ){
  ros::init(argc,argv,"speak_to_people");
  if( !ros::master::check() )  return 1;

  Speak sp;
  ros::spin();

  return(0);
}
