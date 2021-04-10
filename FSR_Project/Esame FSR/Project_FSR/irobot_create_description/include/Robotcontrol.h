#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <algorithm>
#include <math.h> 
#include "geometry_msgs/Pose.h"
#include <actionlib/client/simple_action_client.h>
#include "boost/thread.hpp"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <fstream>
#include <nav_msgs/Odometry.h> 
#include <sensor_msgs/LaserScan.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>

using namespace std;
	
#define PI 3.14159265
#define RAD 0.017453292
#define PII 6.28318530
#define Rotation 0.2
#define Speed 0.2
#define LIN_VEL 0.2
#define ANG_VEL 0.2

struct Repulsive {
	float Pcos = 0;
	float Psin = 0;
	float Fcos = 0;
	float Fsin = 0;

} ;

struct Robot {
	float Pos_x = 0;
	float Pos_y = 0;
	float X_B = 0;
	float Y_B = 0;
	float Theta_z = 0;
	float Theta_w = 0;
	float Rad_Theta = 0;
	float Deg_Theta = 0;
	float Forvel = 0;
	float Rotvel = 0;
	float VelLeft = 0;
	float VelRight = 0;
	float Rangevision = 50;
	float Influence = 5;
	float Dist_max = 0;
	float Dist_min = 50;
	float V_dx = 0;
	float V_dy = 0;
	float K1 = 0;
	float K2 = 0;
};

struct Laser {
	
	float Dist = 0;
	float Rad_Theta = 0;
	float Deg_Theta = 0;
	float Cos = 0;
	float Sin = 0;
	float Lunghezza = 0;
	int Angolo = 0;
	float Deepth = 0;
	int Angle = 0;
};

struct Goal {
	float Pos_x = 0;
	float Pos_y = 0;
	float Deg = 0;
	float Pos_x_t = 0;
	float Final_x = 0;
	float Pos_y_t = 0;
	float Final_y = 0;
	float Dist = 0;
	float Rad_Theta = 0;
	float Deg_Theta = 0;
	float Diff_Rad = 0;
};

struct Totale {
	float RPcos = 0;
	float RPsin = 0;
	float RFcos = 0;
	float RFsin = 0;
	float APcos = 0;
	float APsin = 0;
	float AFcos = 0;
	float AFsin = 0;
	float Pcos = 0;
	float Psin = 0;
	float Fcos = 0;
	float Fsin = 0;
	float RPRadice = 0;
	float RFRadice = 0;
	float APRadice = 0;
	float AFRadice = 0;
	float PRadice = 0;
	float FRadice = 0;
};

struct Odometr {
	float Pos_x = 0;
	float Pos_y = 0;
	float Theta = 0;
	float Delta_R = 0;
	float Delta_L = 0;
	float Forvel = 0;
	float Rotvel = 0;
	float Error_x = 0;
	float Error_y = 0;
	float Error_theta = 0;
};



Repulsive Rep [360];
Robot Bot [1];
Laser L [360];
Goal G[1];
Totale Tot[1];
Odometr Odome[1];

class hospitalRobot {


	private:
		ros::NodeHandle _nh;
		ros::Subscriber _odom_bot;
		ros::Subscriber _scan_bot;
		ros::Publisher _wheel_pub;
		ros::Publisher _error_pub;
		ros::Publisher _PotFor_pub;
		ros::Publisher _vel_pub;
		ros::Publisher marker_pub;
	
	public:
		hospitalRobot();
		uint32_t shape = visualization_msgs::Marker::ARROW;
		float Angle_Min = 0;
		float bi = -1;
		int Param = 1;
		float T_s = 0.01;
		bool Contact = false;
		int Laser_sector = 360;
		float K_RepPot = 0.001;	
		float K_AttPot = 0.1;
		float k_1 = 0.05;
		float k_2 = 0.05;
		int r =0;
		bool Find_Depth = false;
	
		float DistWheel = 0.24;
		float RadWheel = 0.03;
		float Angulum = 0;
		float Angulum_1 = 0;
		float Angulum_2 = 0;
		int num =0;
		bool Avanti = false;
		float Angolo_falso_1 = 0;
		float Angolo_falso_2 = 0;
		float Angolo_falso = 0;
		float Long=0;
		float Long_1=0;
		float Long_2=0;
		float Max_Long = 0;
		bool Val = true;
		int LatoA = 0;
		int LatoB = 0;
		int AngoloAB = 0;
		float Larghezza = 0;
		int Grado = 0;
		int Angnum_s = 0;
		int Angnum_d = 0;
		int Angnum = 0;
		float PuntoX = 0;
		float PuntoY = 0;
		float Radice = 0;
		float Radice_1 = 0;
		float Angolo_Forza = 0;
		float Angolo_Forza_1 = 0;
		float Limite = 4.7;
		bool onlyone = true;
		bool stop = true;
		void Inizialization ();
		void run ();
		void odom_cb( nav_msgs::OdometryConstPtr odom );
		void laser_cb ( sensor_msgs::LaserScanConstPtr laser );
		void Repulsive_Potential ();
		void Attractive_Potential ();
		void Total_Potential ();
		void Go_Rotation ();
		void Center_Laser ();
		void Correzioneangolo ();
		void Correzionesettore ();
		void Print ();
		void Strettoia () ;
		void Odometry ();
		void CambioGoal ();



		

};
