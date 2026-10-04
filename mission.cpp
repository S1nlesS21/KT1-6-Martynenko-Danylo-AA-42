#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>  
#include <nav_msgs/Odometry.h>          

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>


struct Vec3
{
  double x, y, z;
};

struct Waypoint
{
  std::string name;  
  int id;            
  Vec3 pos;          
};

struct Segment
{
  std::string from, to;  
  double time;           
  double length;         
  bool reached;          
};

double dist3(const Vec3 &a, const Vec3 &b)  
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                   (a.z - b.z) * (a.z - b.z));
}

double distXY(const Vec3 &a, const Vec3 &b)  
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

 // Вузол

class Mission
{
public:
  Mission() : pnh_("~") {}
  bool init();  
  void run();   

private:
  bool readPoint(const std::string &name, int id);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void publishGoal(const Waypoint &wp);
  Segment flyTo(const Waypoint &wp, const std::string &from);
  void printReport(bool finished);

  ros::NodeHandle nh_;   
  ros::NodeHandle pnh_;  
  ros::Publisher goal_pub_;
  ros::Subscriber odom_sub_;

  // Параметри
  double tolerance_, hold_time_, flight_height_, goal_timeout_;
  int max_attempts_;
  std::string goal_topic_, odom_topic_, csv_path_;

  std::vector<Waypoint> points_;  // A, B, C, D

  // Стан
  Vec3 pos_{0.0, 0.0, 0.0};   
  bool have_odom_ = false;    
  bool on_route_ = false;     
  double path_length_ = 0.0;  
  ros::Time t_start_;         
  ros::Time t_end_;           

  std::ofstream csv_;
  std::vector<Segment> segments_;
};

// Ініціалізація

bool Mission::init()
{
  pnh_.param("tolerance", tolerance_, 0.5);    
  pnh_.param("hold_time", hold_time_, 1.0);    
  pnh_.param("flight_height", flight_height_, 1.0);  
  pnh_.param("goal_timeout", goal_timeout_, 40.0);   
  pnh_.param("max_attempts", max_attempts_, 2);      
  pnh_.param<std::string>("goal_topic", goal_topic_, "/move_base_simple/goal");
  pnh_.param<std::string>("odom_topic", odom_topic_, "/drone_0_visual_slam/odom");

  // Точки маршруту
  if (!readPoint("A", 0) || !readPoint("B", 1) || !readPoint("C", 2) || !readPoint("D", 3))
    return false;

  char stamp[32];
  std::time_t now = std::time(nullptr);
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
  const char *home = std::getenv("HOME");
  std::string default_csv = std::string(home ? home : ".") + "/mission_" + stamp + ".csv";
  pnh_.param<std::string>("csv_path", csv_path_, default_csv);

  csv_.open(csv_path_);
  if (!csv_.is_open())
  {
    ROS_ERROR("Не вдалося створити файл %s", csv_path_.c_str());
    return false;
  }
  csv_ << std::fixed << std::setprecision(3);
  csv_ << "time_s,x,y,z,goal,dist_m\n";

  // Зв'язок із планувальником: публікуємо цілі, слухаємо одометрію
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);
  odom_sub_ = nh_.subscribe(odom_topic_, 10, &Mission::odomCallback, this);
  return true;
}

bool Mission::readPoint(const std::string &name, int id)
{
  std::vector<double> v;
  if (!pnh_.getParam(name, v) || v.size() != 3)
  {
    ROS_ERROR("Точку %s не знайдено в параметрах (очікується %s: [x, y, z])",
              name.c_str(), name.c_str());
    return false;
  }
  Waypoint wp{name, id, {v[0], v[1], v[2]}};
  ROS_INFO("Точка %s: (%.2f, %.2f, %.2f)", name.c_str(), wp.pos.x, wp.pos.y, wp.pos.z);

  if (std::fabs(wp.pos.z - flight_height_) > 1e-3)
    ROS_WARN("Точка %s: z = %.2f буде проігноровано, планувальник летить на %.2f м",
             name.c_str(), wp.pos.z, flight_height_);

  points_.push_back(wp);
  return true;
}

// Одометрія 

void Mission::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  Vec3 p{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};

  if (have_odom_ && on_route_)
    path_length_ += dist3(pos_, p);

  pos_ = p;
  have_odom_ = true;
}

// Публікація цілі

void Mission::publishGoal(const Waypoint &wp)
{
  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "world";
  goal.pose.position.x = wp.pos.x;
  goal.pose.position.y = wp.pos.y;
  goal.pose.position.z = flight_height_;  
  goal.pose.orientation.w = 1.0;          

  goal_pub_.publish(goal);
  ROS_INFO("-> Ціль %s: (%.2f, %.2f)", wp.name.c_str(), wp.pos.x, wp.pos.y);
}

// Політ до однієї точки 

Segment Mission::flyTo(const Waypoint &wp, const std::string &from)
{
  const Vec3 target{wp.pos.x, wp.pos.y, flight_height_};
  const ros::Time seg_start = ros::Time::now();
  const double len_start = path_length_;

  publishGoal(wp);
  ros::Time last_pub = seg_start;
  int attempts = 1;

  bool inside = false;     
  ros::Time inside_since;  
  bool reached = false;    
  bool gave_up = false;    
  ros::Time seg_end = seg_start;

  ros::Rate rate(10);  
  while (ros::ok())
  {
    ros::spinOnce();  
    const ros::Time now = ros::Time::now();
    const double d = dist3(pos_, target);
    seg_end = now;

    csv_ << (now - t_start_).toSec() << ',' << pos_.x << ',' << pos_.y << ',' << pos_.z
         << ',' << wp.id << ',' << d << '\n';

    if (d < tolerance_)
    {
      if (!inside)
      {
        inside = true;
        inside_since = now; 
      }
      else if ((now - inside_since).toSec() >= hold_time_)
      {
        reached = true;
      }
    }
    else
    {
      inside = false;  
    }

    if (!reached && (now - last_pub).toSec() > goal_timeout_)
    {
      if (attempts < max_attempts_)
      {
        ROS_WARN("Ціль %s: не долетіли за %.0f с, публікую ще раз", wp.name.c_str(), goal_timeout_);
        publishGoal(wp);
        last_pub = now;
        ++attempts;
      }
      else
      {
        ROS_ERROR("Ціль %s недосяжна (можливо, всередині перешкоди), пропускаю", wp.name.c_str());
        gave_up = true;
      }
    }

    rate.sleep();  
    if (reached || gave_up)
      break;
  }

  t_end_ = seg_end;
  Segment s{from, wp.name, (seg_end - seg_start).toSec(), path_length_ - len_start, reached};
  if (reached)
    ROS_INFO("Досягнуто %s: %.1f с, %.1f м", wp.name.c_str(), s.time, s.length);
  return s;
}

// Увесь маршрут 

void Mission::run()
{
  ros::Rate rate(10);

  ROS_INFO("Чекаю одометрію з %s ...", odom_topic_.c_str());
  while (ros::ok() && !have_odom_)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Чекаю планувальник на %s (потрібен flight_type = 1) ...", goal_topic_.c_str());
  while (ros::ok() && goal_pub_.getNumSubscribers() == 0)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ros::Duration(0.5).sleep();  
  if (!ros::ok())
    return;

  const Waypoint &A = points_[0];
  std::vector<Waypoint> route;
  std::string from = "A";
  if (distXY(pos_, A.pos) > tolerance_)
  {
    ROS_WARN("Дрон за %.1f м від точки A, спершу летимо в A", distXY(pos_, A.pos));
    route.push_back(A);
    from = "start";
  }
  route.push_back(points_[1]);
  route.push_back(points_[2]);
  route.push_back(points_[3]);
  route.push_back(A);

  // Виконуємо маршрут
  ROS_INFO("Старт маршруту. CSV: %s", csv_path_.c_str());
  t_start_ = ros::Time::now();
  t_end_ = t_start_;
  on_route_ = true;
  for (const Waypoint &wp : route)
  {
    if (!ros::ok())
      break;
    segments_.push_back(flyTo(wp, from));
    from = wp.name;
  }
  on_route_ = false;

  printReport(ros::ok());  
}

// Підсумки

void Mission::printReport(bool finished)
{
  const double total = (t_end_ - t_start_).toSec();

  std::printf("\n========== %s ==========\n", finished ? "МАРШРУТ ЗАВЕРШЕНО" : "МАРШРУТ ПЕРЕРВАНО");
  std::printf("Відрізок        Час, с    Шлях, м\n");
  for (const Segment &s : segments_)
    std::printf("%-12s%10.1f%11.1f%s\n", (s.from + " -> " + s.to).c_str(), s.time, s.length,
                s.reached ? "" : "   (не досягнуто)");
  std::printf("-----------------------------------\n");
  std::printf("Загальний час маршруту:   %.1f с\n", total);
  std::printf("Довжина пройденого шляху: %.1f м\n", path_length_);
  if (total > 0.0)
    std::printf("Середня швидкість:        %.2f м/с\n", path_length_ / total);
  std::printf("CSV-лог: %s\n\n", csv_path_.c_str());
  std::fflush(stdout);
}

// main

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission");
  Mission mission;
  if (!mission.init())
    return 1;
  mission.run();
  return 0;
}
