#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/io/pcd_io.h>
#include <sensor_msgs/image_encodings.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

// #define DEPTH_FROM_POINTS

const int WIDTH = 640;
const int HEIGHT = 480;
const int MY_BUFFER_SIZE = 5 * WIDTH * HEIGHT; // 3 * 640 * 480 + 2 * 640 * 480;
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
  ros::Subscriber _sub1, _sub2;
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  mySocketClient msc;
  cv::Mat depth_img;
  char *buffer;
public:
  ViewImage() {
    _sub1 = _nh.subscribe ("/camera/rgb/image_color", 1,  &ViewImage::image_cb, this);
    ROS_INFO ("Listening for incoming data on topic /camera/rgb/image_color ..." );
#ifdef DEPTH_FROM_POINTS
    _sub2 = _nh.subscribe ("/camera/depth_registered/points", 1,  &ViewImage::points_cb, this);
    ROS_INFO ("Listening for incoming data on topic /camera/rgb/points ..." );
#else
    _sub2 = _nh.subscribe ("/camera/depth/image_raw", 1,  &ViewImage::image2_cb, this);
    ROS_INFO ("Listening for incoming data on topic /camera/depth/image_raw ..." );
#endif

    msc.init( PORT, HOSTNAME );

    // create depth img
    depth_img = cv::Mat( HEIGHT, WIDTH, CV_16U );

    // create buffer
    buffer = new char[ MY_BUFFER_SIZE ];
  }

  ~ViewImage() {
    delete [] buffer;
  }

  //* get points
  void points_cb( const sensor_msgs::PointCloud2ConstPtr& cloud ){
    if ((cloud->width * cloud->height) == 0)
      return;
    pcl::fromROSMsg (*cloud, input_cloud);
  }

  //* get depth img
  void image2_cb( const sensor_msgs::ImageConstPtr& msg2 ){
    cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(msg2);
    depth_img = cv_ptr2->image;
  }

  //* send color img and depth img
  void image_cb( const sensor_msgs::ImageConstPtr& msg ){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
    //cv::Mat color_img = cv_ptr->image;

#ifdef DEPTH_FROM_POINTS
    if ((input_cloud.width * input_cloud.height) == 0)
      return;
    // create depth img
    for(int r = 0; r < depth_img.rows; ++r)
      for(size_t c = 0; c < (size_t)depth_img.cols; ++c)
	depth_img.at<uint16_t>( r, c ) = int( 1000 * input_cloud.points[ r * depth_img.cols + c ].z );
#endif

    memcpy( buffer, (char *)cv_ptr->image.data, 3 * WIDTH * HEIGHT );
    memcpy( buffer + 3 * WIDTH * HEIGHT, (char *)depth_img.data, 2 * WIDTH * HEIGHT );
    msc.send( buffer );
  }

};

int main( int argc, char** argv ){
  ros::init(argc,argv,"send_color_and_depth_img");
  if( !ros::master::check() )  return 1;

  ViewImage view;
  ros::spin();

  return(0);
}
