#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

const int MY_BUFFER_SIZE = 3 * 640 * 480;
const int BUFFER_LEN = 512;
const int PORT = 27015;
const char* HOSTNAME = "10.137.70.26";

class mySocketClient {
 private:
  int sockfd;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  
  void error(const char *msg)
  {
    perror(msg);
    exit(0);
  }
  
 public:
  mySocketClient(){}

  mySocketClient( int portno, const char* hostname ){
    init( portno, hostname );    
  }

  void init( int portno, const char* hostname ){
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    struct timeval tv;
    tv.tv_sec = 10;  /* 10 Secs Timeout */
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO,(struct timeval *)&tv,sizeof(struct timeval));
    tv.tv_sec = 10;  /* 10 Sec Timeout */
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,(struct timeval *)&tv,sizeof(struct timeval));

    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
  }

  int send( char* buffer ){

      int n = write(sockfd,buffer,MY_BUFFER_SIZE);

      if (n < 0) 
	error("ERROR writing to socket");

      bzero(buffer,MY_BUFFER_SIZE);
      n = read(sockfd,buffer,BUFFER_LEN);
      if (n < 0) 
	error("ERROR reading from socket");
#if 0
      printf("%s\n",buffer);
#else
      // bbox
      int len = (int)buffer[ 0 ];
      buffer++;
      uint16_t* bbox = (uint16_t *)buffer;

      if (len > 0)
	std::cout << "Tracking: " << len << " people." << std::endl;
      for( int i=0; i<len; i++ ){
	int x1 = bbox[ i * 4 ];
	int y1 = bbox[ i * 4 + 1 ];
	int x2 = bbox[ i * 4 + 2 ];
	int y2 = bbox[ i * 4 + 3 ];
	std::cout << "     " << x1 << " " << y1 << " | " << x2 << " " << y2 << std::endl;
      }
#endif

      // // close and open
      // close(sockfd);
      // sockfd = socket(AF_INET, SOCK_STREAM, 0);
      // if (sockfd < 0) 
      //   error("ERROR opening socket");
      // if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
      //   error("ERROR connecting");

      return 1;
  }

  ~mySocketClient(){
    close(sockfd);
  }

};

class ViewImage {
private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  mySocketClient msc;
public:
  ViewImage() {
    _sub = _nh.subscribe ("/camera/rgb/image_color", 1,  &ViewImage::image_cb, this);
    ROS_INFO ("Listening for incoming data on topic /camera/rgb/image_color ..." );
    msc.init( PORT, HOSTNAME );
  }

  ~ViewImage() {}

  //* show color img
  void image_cb( const sensor_msgs::ImageConstPtr& msg ){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
    //cv::Mat color_img = cv_ptr->image;

    msc.send( (char *)cv_ptr->image.data );
  }

};

int main( int argc, char** argv ){
  ros::init(argc,argv,"send_color_img");
  if( !ros::master::check() )  return 1;

  ViewImage view;
  ros::spin();

  return(0);
}
