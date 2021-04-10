#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "boost/thread.hpp"

#include <iostream>

using namespace std;

#define LIN_VEL 0.1
#define ANG_VEL 0.1
#define Rad 0.03
#define Dist 0.24

class KEY_CTRL {
	public:
		KEY_CTRL();
		void key_input();
		void run();
		void vel_ctrl();

	private:
		ros::NodeHandle _nh;
		ros::Publisher  _vel_pub;

		float _fv; //Forward velocity	
		float _rv; //Rotational velocity

};


KEY_CTRL::KEY_CTRL() {	
	_vel_pub = _nh.advertise< geometry_msgs::Twist >("/robot/cmd_vel", 100);

}


void KEY_CTRL::key_input() {

	string input;

	cout << "Keyboard Input: " << endl;
	cout << "[w]: Forward direction velocity" << endl;
	cout << "[x]: Backward direction velocity" << endl;
	cout << "[a]: Left angular velocity" << endl;
	cout << "[d]: Right angular velocity" << endl;

	cout << "[s]: stop the robot!" << endl;

	while (ros::ok()) {

		getline( cin, input);

		if( input == "w" ) {
			_fv = (_fv < 0.0 ) ? 0.0 : LIN_VEL;}

		else if( input == "x" ) {
			_fv = (_fv > 0.0 ) ? 0.0 : -LIN_VEL;}

		else if( input == "a" ) {
			_fv =   10*Rad/2 ;
			_rv =	10*Rad/Dist ;}

			//_rv = (_rv > 0.0 ) ? 0.0 : -ANG_VEL;
		else if( input == "d" ){
			_fv =   10*Rad/2 ;
			_rv =	-10*Rad/Dist ;}

			//_rv = (_rv < 0.0 ) ? 0.0 : ANG_VEL;
		else if( input == "s" ) {
			_fv = _rv = 0.0;}

	}
}


void KEY_CTRL::vel_ctrl() {
	
	ros::Rate r(10);
	geometry_msgs::Twist cmd_vel;

	while(ros::ok()) {

		cmd_vel.linear.x = _fv;
		cmd_vel.angular.z = _rv;
		_vel_pub.publish( cmd_vel );

		r.sleep();
	}

}


void KEY_CTRL::run() {
	boost::thread key_input_t( &KEY_CTRL::key_input, this );
	boost::thread vel_ctrl_t( &KEY_CTRL::vel_ctrl, this );
	ros::spin();
}


int main(int argc, char** argv ) {

	ros::init(argc, argv, "key_ctrl");

	KEY_CTRL kc;
	kc.run();

	return 0;
}
