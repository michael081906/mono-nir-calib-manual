#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <dynamic_reconfigure/server.h>
#include <mono_nir_calib_manual/mono_nir_calib_cfgConfig.h>

double tx,ty,tz,rx,ry,rz;
double tx_ini,ty_ini,tz_ini,rx_ini,ry_ini,rz_ini;
double PI = 3.14159;

void callback_d(mono_nir_calib_manual::mono_nir_calib_cfgConfig &config, uint32_t level) {
 ROS_INFO("Reconfigure Request: %f %f %f - %f %f %f",
            config.x, config.y,config.z,
            config.roll, config.pitch,config.yaw);

            tx = config.x;
            ty = config.y;
            tz = config.z;
            rx = PI/180 * config.roll;
            ry = PI/180 * config.pitch;
            rz = PI/180 * config.yaw;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "mono_nir_calib_manual");
  ros::NodeHandle node;
  ros::NodeHandle home("~");
  home.getParam("rx_launch", rx_ini);
  home.getParam("ry_launch", ry_ini);
  home.getParam("rz_launch", rz_ini);
  home.getParam("x_launch", tx_ini);
  home.getParam("y_launch", ty_ini);
  home.getParam("z_launch", tz_ini);
  static tf::TransformBroadcaster br;
  dynamic_reconfigure::Server<mono_nir_calib_manual::mono_nir_calib_cfgConfig> server;
  dynamic_reconfigure::Server<mono_nir_calib_manual::mono_nir_calib_cfgConfig>::CallbackType f;
  f = boost::bind(&callback_d, _1, _2);
  server.setCallback(f);
  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  double x = 0.0; 
  double y = 0.0;
  double z = 0.0; 
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 0.0; 

  while(ros::ok())
  {
  tf::Transform transform;
  x = tx_ini + tx;
  y = ty_ini + ty;
  z = tz_ini + tz;
  transform.setOrigin( tf::Vector3(x, y, z) );
  tf::Quaternion q;
  q.setRPY(rx_ini + rx, ry_ini + ry, rz_ini + rz);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "stage", "mono"));
  std::cout << x<< " "<< y << " "<< z<< " "<< q.x() << " " <<q.y()<< " "<< q.z() << " " << q.w()<<std::endl;		

  loop_rate.sleep();
  ros::spinOnce();
}
  return 0;
};
