// file: dotnect_dgram_socket.cpp, style: README.me
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016 14U2g4ocMy5aB2cY4cmCtbXD6qyNQzujuA (serves donations as well)
// All rights reserved.
//
// dotnect_dgram_socket receives sensor data and dynamically publishes on ros topics
// expected socket input: "<timestamp> <type> <...> "
//
#include <arpa/inet.h>
#include <fcntl.h>
#include <cerrno>
#include <ctime>
#include <ifaddrs.h>
#include <iomanip>
#include <netinet/in.h>
#include <signal.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
// boost
#include <boost/algorithm/string.hpp>
#include <boost/atomic.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
// ros
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>

#define NNAME "dotnect_dgram_socket: "
#define N 14

using namespace std;

// keep track of datagram numbers
int g_dgram_count[N];
// incoming verbose signal
boost::atomic<bool> g_verbose(false);

// output user args
void printInfo()
{
  printf("\x1b[36m$ dotnect_dgram_socket <1:string> <2:int> <3:string> <4:int>\x1b[0m\n");
  printf("    1:multicastgroupIP 2:multicastgroupPort\n");
  printf("    3:localInterfaceName  use $ ifconfig to get interface name on "
         "which to join the network\n");
  printf("    4:be_verbose          0:silent 1:print all DPs to command-line\n");
}

// react to incoming verbose signal
bool setVerbose(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  g_verbose = req.data;
  res.success = true;
  res.message = "ok";
  return true;
}

// answer service calls
void ros_spin()
{
  ros::spin();
}

// one object per topic embracing publisher and message generation
class PublishTopic
{
public:
  PublishTopic(string topic_name, int type) : type(type)
  {
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>(topic_name, 10);
    ROS_INFO(NNAME "sensor type %d on %s", type, topic_name.c_str());
    // prepare standard message layout
    msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_.layout.dim[0].stride = 1;
    msg_.layout.dim[0].label = topic_name;
  }
  ~PublishTopic()
  {
  }
  void publishVector(vector<double> &vec)
  {
    msg_.layout.dim[0].size = vec.size();
    msg_.data = vec;
    pub_.publish(msg_);
  }
  int type;

private:
  std_msgs::Float64MultiArray msg_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

// handles currently active topics.
// depending on the sensor data topics (un)advertise on the go
class PublishTopicHandler
{
public:
  PublishTopicHandler()
  {
    topic_names_.push_back("");
    topic_names_.push_back("/dotnect_dgram_socket/acc");  // 1
    topic_names_.push_back("/dotnect_dgram_socket/mag");
    topic_names_.push_back("/dotnect_dgram_socket/orient");
    topic_names_.push_back("/dotnect_dgram_socket/gyro");
    topic_names_.push_back("/dotnect_dgram_socket/light");  // 5
    topic_names_.push_back("/dotnect_dgram_socket/pres");
    topic_names_.push_back("/dotnect_dgram_socket/temp");
    topic_names_.push_back("/dotnect_dgram_socket/prox");
    topic_names_.push_back("/dotnect_dgram_socket/grav");
    topic_names_.push_back("/dotnect_dgram_socket/linacc");  // 10
    topic_names_.push_back("/dotnect_dgram_socket/rot");
    topic_names_.push_back("/dotnect_dgram_socket/humid");
    topic_names_.push_back("/dotnect_dgram_socket/ambtmp");
  }
  ~PublishTopicHandler()
  {
  }
  // advertise and publish topic
  void handleVector(vector<double> &vec)
  {
    boost::mutex::scoped_lock lock(mutex_);
    int type = vec[1];
    int position = getObjPosition(type);
    if (position != -1)
    {
      topics_vector_[position].publishVector(vec);
    }
    else
    {
      topics_vector_.push_back(PublishTopic(topic_names_[type], type));
      topics_vector_.back().publishVector(vec);
      // update current topics online
      g_dgram_count[0] = topics_vector_.size();
    }
  }
  // unadvertise topic
  int unadvertiseTopic(int type)
  {
    boost::mutex::scoped_lock lock(mutex_);
    int position = getObjPosition(type);
    if (position != -1)
    {
      topics_vector_.erase(topics_vector_.begin() + position);
    }
    return topics_vector_.size();
  }
  // get topic name
  string getTopicName(uint type)
  {
    if (type >= topic_names_.size())
    {
      return "name not available";
    }
    return topic_names_[type];
  }
private:
  // locate object in vector
  int getObjPosition(int type)
  {
    for (int i = 0; i < topics_vector_.size(); ++i)
    {
      if (topics_vector_[i].type == type)
      {
        return i;
      }
    }
    return -1;  // not existent
  }
  // member variables  (fields)
  boost::mutex mutex_;
  vector<PublishTopic> topics_vector_;
  vector<string> topic_names_;
};

// static global object
PublishTopicHandler *getPublishTopicHandler()
{
  static PublishTopicHandler topic_handler;
  return &topic_handler;
}

//  trigger unadvertising topics depending on datagram types
void checkPublishTopicHandler()
{
  int dgram_count_bef[N];
  int dgram_count_dif[N];
  // timespan we guarantee to not unadvertiseTopic topic
  struct timespec req, rem;
  req.tv_sec = 5;   //  seconds
  req.tv_nsec = 0;  //  nanoseconds

  for (int i = 1; i < N; ++i)
  {
    dgram_count_dif[i] = 0;
    dgram_count_bef[i] = g_dgram_count[i];
  }

  while (nanosleep(&req, &rem) != -1)
  {
    for (int i = 1; i < N; ++i)
    {
      // DPs incoming
      if (g_dgram_count[i] > 0)
      {
        dgram_count_dif[i] = g_dgram_count[i] - dgram_count_bef[i];
        dgram_count_bef[i] = g_dgram_count[i];
        if (g_verbose)
        {
          printf("\x1b[33m" NNAME "rate %s at %d Hz \x1b[0m\n", getPublishTopicHandler()->getTopicName(i).c_str(),
                 dgram_count_dif[i] / 5);
        }
        // DPs stopped incoming
        if (dgram_count_dif[i] == 0)
        {
          if (getPublishTopicHandler()->unadvertiseTopic(i) == 0)
          {
            ROS_WARN(NNAME "unadvertised topic %s", getPublishTopicHandler()->getTopicName(i).c_str());
            ROS_WARN(NNAME "no topic datagrams. Retrying...");
          }
          else
          {
            ROS_WARN(NNAME "unadvertised topic %s", getPublishTopicHandler()->getTopicName(i).c_str());
          }
          g_dgram_count[i] = 0;
        }
      }
    }
  }
}

// Map Interface name to Interface-IP4
bool getInterfaceIP4(string ifa_name_str, string &ifa_ip4)
{
  struct ifaddrs *ifa_addr_struct = NULL;
  struct ifaddrs *ifa = NULL;
  void *addrPtr = NULL;
  getifaddrs(&ifa_addr_struct);

  for (ifa = ifa_addr_struct; ifa != NULL; ifa = ifa->ifa_next)
  {
    if (!ifa->ifa_addr)
    {
      continue;
    }
    if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name, ifa_name_str.c_str()) == 0)
    {
      // IP4 and Name ok
      addrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
      char buff[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, addrPtr, buff, INET_ADDRSTRLEN);
      ifa_ip4 = buff;
      return true;
    }
  }
  return false;
}

// pauses caller
bool customSleep(int sec, int nsec)
{
  struct timespec req, rem;
  req.tv_sec = sec;
  req.tv_nsec = nsec;
  if (nanosleep(&req, &rem) != 0)
  {
    printf("\x1b[31m" NNAME "rem %d:%d strerror:%s \x1b[0m\n", (int)rem.tv_sec, (int)rem.tv_nsec, strerror(errno));
    return false;
  }
  return true;
}

// get datagrams and publish on sensor_topics
void startSocket(string group_ip4, int group_port, string ifa_name)
{
  // file descriptor for socketadress family, socket type and protocol type
  // doc error types http://man7.org/linux/man-pages/man2/socket.2.html
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
  {
    ROS_ERROR(NNAME "%s", strerror(errno));
    return;
  }
  // allow multiple sockets on the same host to bind to the same adress
  int optval = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) < 0)
  {
    ROS_ERROR(NNAME "%s", strerror(errno));
    close(fd);
    return;
  }
  // Bind to port and IP as specified in INADDR_ANY.
  struct sockaddr_in localSock;
  memset((char *)&localSock, 0, sizeof(localSock));
  localSock.sin_family = AF_INET;
  localSock.sin_port = htons(group_port);
  localSock.sin_addr.s_addr = INADDR_ANY;
  if (bind(fd, (struct sockaddr *)&localSock, sizeof(localSock)))
  {
    ROS_ERROR(NNAME "%s", strerror(errno));
    close(fd);
    return;
  }
  // Join multicast group on local interface
  struct ip_mreq group;
  group.imr_multiaddr.s_addr = inet_addr(group_ip4.c_str());
  string itfIP4 = "";
  if (!getInterfaceIP4(ifa_name, itfIP4))
  {
    ROS_ERROR(NNAME "Interface name %s not solved (check ifconfig)", ifa_name.c_str());
    close(fd);
    return;
  }
  group.imr_interface.s_addr = inet_addr(itfIP4.c_str());
  if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) == 0)
  {
    ROS_INFO(NNAME "Joined multicast group on %s %s", ifa_name.c_str(), itfIP4.c_str());
  }
  else
  {
    ROS_ERROR(NNAME "%s (Check ifconfig)", strerror(errno));
    close(fd);
    return;
  }

  char databuf[150];
  vector<string> sensor_data_str;
  vector<double> sensor_data;
  ROS_WARN(NNAME "No Datagrams. Retrying...");
  // if no avail data read shall return -1
  int existing_v_dbls_flags_ = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, existing_v_dbls_flags_ | O_NONBLOCK);

  while (!ros::isShuttingDown())
  {
    int read_flag = read(fd, databuf, sizeof(databuf));
    if (read_flag >= 0)
    {
      sensor_data_str.clear();
      boost::split(sensor_data_str, databuf, boost::is_any_of(" "));
      sensor_data_str.erase(sensor_data_str.begin() + 1);
      sensor_data.resize(sensor_data_str.size());
      // convert string vector to double vector
      int j = 0;
      vector<string>::const_iterator itr;
      for (itr = sensor_data_str.begin(); itr != sensor_data_str.end(); ++itr)
      {
        istringstream iss(*itr);
        iss >> sensor_data[j++];
      }
      // throw out bad boost::split conversions
      int type = sensor_data[1];
      if (type > 0 && type < N)
      {
        g_dgram_count[type]++;
        getPublishTopicHandler()->handleVector(sensor_data);
        if (g_verbose)
        {
          printf("%s\n", databuf);
        }
      }
      else
      {
        printf(NNAME "type %d, discarded '%s'", type, databuf);
      }
    }
    else if (read_flag < 0 && errno == EAGAIN)
    {
      // no data to be read
      customSleep(0, 1000000);
    }
    else
    {
      ROS_ERROR(NNAME "%s", strerror(errno));
    }
  }
  close(fd);
}

// check user args and start socket thread
int main(int argc, char **argv)
{
  if (argc < 5)
  {
    printInfo();
    exit(0);
  }
  // shell args
  string ifa_name;
  string group_ip4;
  int group_port = 0;
  bool tmp;
  group_ip4 = argv[1];
  istringstream(argv[2]) >> group_port;
  ifa_name = argv[3];
  istringstream(argv[4]) >> tmp;
  g_verbose = tmp;

  // wait for roslaunch server to prevent nodes entangling printf
    customSleep(3, 0);

  // provide command line arguments and node name to ROS
  ros::init(argc, argv, "dotnect_dgram_socket");
  ros::NodeHandle nh_;

  // react to incoming verbose signal
  ros::ServiceServer service = nh_.advertiseService("dotnect_dgram_socket_service", setVerbose);
  boost::thread t_ros_spin(ros_spin);

  // open socket and add/delete topics
  boost::thread t_start_socket(startSocket, group_ip4, group_port, ifa_name);
  checkPublishTopicHandler();

  // shutdown handling
  t_ros_spin.join();
  printf(NNAME "thread t_ros_spin terminated \n");
  t_start_socket.join();
  printf(NNAME "thread t_start_socket terminated \n");
  printf(NNAME "returned.\n");
  return 0;
}
// EOF
