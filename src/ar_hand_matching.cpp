#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

using namespace message_filters;

class ARHandMatch
{
public:
  ARHandMatch();
  ~ARHandMatch();
private:
  void matchingCallback(const geometry_msgs::Point& lefthand_point, const geometry_msgs::Point& ar_point_2d);

  // publisher
  ros::Publisher decision_pub;//フラグ立てたりする？
  ros::Publisher hand_pub;
  // subscriber (point)
  // ros::Subscriber ar_sub;
  // ros::Subscriber hand_sub;

  // param
  double image_width;
  double image_hight;
  double width_max;
  double width_min;
  double hight_max;
  double hight_min;
  double th;

  // ros nodehandler
  ros::NodeHandle n;

};

ARHandMatch::ARHandMatch():
  // initialize
  image_width(1024),	//[pix]
  image_hight(768),
  width_max(220),
  width_min(-320),
  hight_max(220),
  hight_min(-320),
  th(0.5)	//[m]
{

  // using parameter server
  n.param("ar_hand_matching/image_width", image_width, image_width);
  n.param("ar_hand_matching/image_hight", image_hight, image_hight);
  n.param("ar_hand_matching/width_max", width_max, width_max);
  n.param("ar_hand_matching/hight_max", hight_max, hight_max);
  n.param("ar_hand_matching/width_min", width_min, width_min);
  n.param("ar_hand_matching/hight_min", hight_min, hight_min);

  hand_pub = n.advertise<geometry_msgs::Point>("cmd_hand", 1);//別のとこで
  decision_pub = n.advertise<std_msgs::Bool>("cmd_check", 1);
  
  // ar_sub = n.subscribe<geometry_msgs::Point>("/*ar-topic*/", 10, &ARHandMatch::matchingCallback, this);
  // hand_sub = n.subscribe<geometry_msgs::Point>("lefthand_point", 10, &ARHandMatch::matchingCallback, this);
}

void ARHandMatch::matchingCallback(const geometry_msgs::Point& lefthand_point, const geometry_msgs::Point& ar_point_2d)
{
  geometry_msgs::Point cmd_hand;
  std_msgs::Bool cmd_check;
  double dis;

  if(lefthand_point.x >= 0){
    cmd_hand.x = (image_width/2)*(1+lefthand_point.x/width_max);
  }
  if(lefthand_point.y >= 0){
    cmd_hand.y = (image_hight/2)*(1+lefthand_point.y/hight_max);
  }
  if(lefthand_point.x < 0){
    cmd_hand.x = (image_width/2)*(1+lefthand_point.x/width_min);
  }
  if(lefthand_point.y < 0){
    cmd_hand.y = (image_hight/2)*(1+lefthand_point.y/hight_min);
  }

  dis = sqrt(pow(cmd_hand.x-ar_point_2d.x, 2)+pow(cmd_hand.y-ar_point_2d.y, 2));
  
  if(dis < th){
    cmd_hand.z = ar_point_2d.z;
    cmd_check.data = 1;
  }else{
    cmd_check.data = 0;
  }

  decision_pub.publish(cmd_check);
  hand_pub.publish(cmd_hand);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_hand_matching");
  ARHandMatch ar_hand_matching;

  message_filters::Subscriber<geometry_msgs::Point> ar_sub(n, "ar_point_2d", 1);
  message_filters::Subscriber<geometry_msgs::Point> hand_sub(n, "lefthand_point", 1);
  TimeSynchronizer<geometry_msgs::Point, geometry_msgs::Point> sync(ar_sub, hand_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
}
