#include "../include/Robotcontrol.h"




hospitalRobot::hospitalRobot(){

	_odom_bot = _nh.subscribe( "/robot/odom", 0, &hospitalRobot::odom_cb, this );
	_scan_bot = _nh.subscribe( "/robot/scan", 0, &hospitalRobot::laser_cb, this );
	_wheel_pub = _nh.advertise< geometry_msgs::Point >("/robot/wheel_vel", 100);
	_error_pub = _nh.advertise< geometry_msgs::Point >("/robot/error", 100);
	_PotFor_pub = _nh.advertise< geometry_msgs::Point >("/robot/PotFor", 100);
	_vel_pub = _nh.advertise< geometry_msgs::Twist >("/robot/cmd_vel", 100);
	marker_pub = _nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
}

void hospitalRobot::Inizialization (){
	
	for (int i=0; i<Laser_sector; i++){
		L[i].Rad_Theta = i*PI/180;
		L[i].Cos = cos (L[i].Rad_Theta);
		L[i].Sin = sin (L[i].Rad_Theta);
	}
	
	Bot[0].Theta_z = 0;
	Bot[0].Theta_w = 0;
	Tot[0].APcos = 0;
	Tot[0].APsin = 0;
	G[0].Pos_x = -4;
	G[0].Pos_y = 4; 
	G[0].Deg = 90;
	num=0;
}

void hospitalRobot::Print (){
	
	num=num+1;
	cout <<"* * * * * * * * * "<< num <<endl;

	if (stop == true){
	cout <<"Pos x : " <<Bot[0].Pos_x<<"     Pos y : "<<Bot[0].Pos_y << endl;
	cout <<"Velocity : " << Bot[0].Forvel <<" m/s     Rotation : "<<Bot[0].Rotvel<<" rad/s"<<endl ;
	cout<< "Velocità ruota sinistra : "<<Bot[0].VelLeft<< " rad/s     Velocità ruota destra : "<<Bot[0].VelRight<<" rad/s"<<endl;
	cout <<"Tot Pot X : "<<Tot[0].Pcos<<"    Tot Pot Y : "<<Tot[0].Psin<< endl;
	cout <<"Att Pot X : "<<Tot[0].APcos <<"    Att Pot Y : "<<Tot[0].APsin<< endl;
	cout <<"Rep Pot X : "<<Tot[0].RPcos <<"    Rep Pot Y : "<<Tot[0].RPsin<< endl;
	cout <<"Forza X :   "<<Tot[0].Fcos<< "   Forza Y : " <<Tot[0].Fsin<< endl;
	cout <<"Dist goal : "<<G[0].Dist<<" m     Angol Goal : "<<G[0].Rad_Theta<<" rad      Angolo Goal : "<<G[0].Deg_Theta<<" gradi"<<endl;
	cout <<"Goal X :    "<< G[0].Pos_x <<"    Goal Y : "<< G[0].Pos_y <<endl;
	cout <<endl;
	}
	else {
	cout<<endl<<endl<<" Destinazione Raggiunta "<<endl<<endl;
	}
	
}

void hospitalRobot::odom_cb( nav_msgs::OdometryConstPtr odom ){

	Bot[0].Pos_x = odom->pose.pose.position.x;
	Bot[0].Pos_y = odom->pose.pose.position.y;
	Bot[0].Theta_z = odom->pose.pose.orientation.z;
	Bot[0].Theta_w = odom->pose.pose.orientation.w;
	
	Correzionesettore ();	

	Bot[0].Rad_Theta = asin (Bot[0].Theta_z)*2;
	Bot[0].Deg_Theta = Bot[0].Rad_Theta*180/PI;
	G[0].Dist= sqrt ((Bot[0].Pos_x-G[0].Pos_x)*(Bot[0].Pos_x-G[0].Pos_x) + (Bot[0].Pos_y-G[0].Pos_y)*(Bot[0].Pos_y-G[0].Pos_y));
	G[0].Rad_Theta = atan2 (G[0].Pos_y-Bot[0].Pos_y, G[0].Pos_x-Bot[0].Pos_x);
	G[0].Deg_Theta = G[0].Rad_Theta*360/(2*PI);
	G[0].Diff_Rad = G[0].Rad_Theta - Bot[0].Rad_Theta;
	Bot[0].X_B = Bot[0].Pos_x + bi*cos (Bot[0].Rad_Theta);
	Bot[0].Y_B = Bot[0].Pos_y + bi*sin (Bot[0].Rad_Theta);

	if (G[0].Pos_x != G[0].Final_x && G[0].Pos_y != G[0].Final_y){
		CambioGoal ();
	}
	
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.ns = "basic_shapes";
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;

	marker.id = r; 
	marker.type = visualization_msgs::Marker::CUBE; 
	
	marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
   
        // Set the color -- be sure to set alpha to something non-zero!

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
	
	marker.pose.position.x = Bot[0].Pos_x;
        marker.pose.position.y = Bot[0].Pos_y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

	marker_pub.publish(marker);

	r=r+1 ;
}



void hospitalRobot::laser_cb ( sensor_msgs::LaserScanConstPtr laser ){
	
	Bot[0].Dist_max	= 0;
	Bot[0].Dist_min = 50;

	for (int i=0; i<Laser_sector; i++){

		L[i].Dist = laser->ranges[i];
		Bot[0].Dist_max = max(Bot[0].Dist_max, L[i].Dist);
		Bot[0].Dist_min = min(Bot[0].Dist_min, L[i].Dist);

		L[i].Rad_Theta = (i*PI/180)+Bot[0].Rad_Theta;
		
		if (L[i].Rad_Theta > PI){		
			L[i].Rad_Theta = L[i].Rad_Theta-PII;
		}
		if (L[i].Rad_Theta < -PI){		
			L[i].Rad_Theta = L[i].Rad_Theta+PII;
		}				
		
		L[i].Cos = cos (L[i].Rad_Theta );
		L[i].Sin = sin (L[i].Rad_Theta );
		
	}

	Strettoia ();	

}



void hospitalRobot::Repulsive_Potential (){

	for (int i = 0; i < Laser_sector; i++){
	
		if (L[i].Dist <= Bot[0].Influence ){
			
			Rep[i].Pcos = 0.5*K_RepPot*(1/L[i].Dist - 1/Bot[0].Influence)*(1/L[i].Dist - 1/Bot[0].Influence)*L[i].Cos; 
			Rep[i].Psin = 0.5*K_RepPot*(1/L[i].Dist - 1/Bot[0].Influence)*(1/L[i].Dist - 1/Bot[0].Influence)*L[i].Sin; 

			Rep[i].Fcos = (K_RepPot/(L[i].Dist*L[i].Dist))*(1/L[i].Dist - 1/Bot[0].Influence)*(L[i].Cos/abs(L[i].Cos));  
			Rep[i].Fsin = (K_RepPot/(L[i].Dist*L[i].Dist))*(1/L[i].Dist - 1/Bot[0].Influence)*(L[i].Sin/abs(L[i].Sin));  
		}

		else{
			Rep[i].Pcos = 0;
			Rep[i].Psin = 0; 	
			Rep[i].Fcos = 0;
			Rep[i].Fsin = 0;
		}	
	}		
}


void hospitalRobot::Attractive_Potential (){

	if (G[0].Dist>1.5){
		Tot[0].APcos = 0.5*K_AttPot*G[0].Dist*G[0].Dist*cos(G[0].Rad_Theta);
		Tot[0].APsin = 0.5*K_AttPot*G[0].Dist*G[0].Dist*sin(G[0].Rad_Theta);
		bi = -1;
		Bot[0].Influence = 2;
	}
	
	else {
		Tot[0].APcos = K_AttPot*G[0].Dist*cos(G[0].Rad_Theta);
		Tot[0].APsin = K_AttPot*G[0].Dist*sin(G[0].Rad_Theta);
		bi = -0.3;
		Bot[0].Influence = 0;
	}

	Tot[0].AFcos = K_AttPot*G[0].Dist*cos(G[0].Rad_Theta);
	Tot[0].AFsin = K_AttPot*G[0].Dist*sin(G[0].Rad_Theta);
} 


void hospitalRobot::Total_Potential (){
	
	geometry_msgs::Point PotFor;	
	Repulsive_Potential ();
	Attractive_Potential ();
	Tot[0].RPcos = 0;
	Tot[0].RPsin = 0;
	Tot[0].RFcos = 0;
	Tot[0].RFsin = 0;

	for (int i = 0; i < Laser_sector; i++){
		
		Tot[0].RPcos = Tot[0].RPcos + Rep[i].Pcos;
		Tot[0].RPsin = Tot[0].RPsin + Rep[i].Psin;
		Tot[0].RFcos = Tot[0].RFcos + Rep[i].Fcos;
		Tot[0].RFsin = Tot[0].RFsin + Rep[i].Fsin;	
		
	}

	Tot[0].Pcos =  Tot[0].APcos - Tot[0].RPcos;
	Tot[0].Psin =  Tot[0].APsin - Tot[0].RPsin;
	Tot[0].Fcos =  Tot[0].AFcos - Tot[0].RFcos;
	Tot[0].Fsin =  Tot[0].AFsin - Tot[0].RFsin;


	Tot[0].RPRadice = sqrt(Tot[0].RPcos*Tot[0].RPcos+Tot[0].RPsin*Tot[0].RPsin);
	Tot[0].RFRadice = sqrt(Tot[0].RFcos*Tot[0].RFcos+Tot[0].RFsin*Tot[0].RFsin);
	Tot[0].APRadice = sqrt(Tot[0].APcos*Tot[0].APcos+Tot[0].APsin*Tot[0].APsin);
	Tot[0].AFRadice = sqrt(Tot[0].AFcos*Tot[0].AFcos+Tot[0].AFsin*Tot[0].AFsin);
	Tot[0].PRadice = Tot[0].APRadice-Tot[0].RPRadice;
	Tot[0].FRadice = Tot[0].AFRadice-Tot[0].RFRadice;
	PotFor.x = Tot[0].PRadice;
	PotFor.y = Tot[0].FRadice;

	Radice = sqrt (Tot[0].Fcos*Tot[0].Fcos+Tot[0].Fsin*Tot[0].Fsin);
	Radice_1 = sqrt (Tot[0].Pcos*Tot[0].Pcos+Tot[0].Psin*Tot[0].Psin);
	Angolo_Forza = atan2 (Tot[0].Fsin,Tot[0].Fcos);
	Angolo_Forza_1 = Angolo_Forza*180/PI;
	
	_PotFor_pub.publish( PotFor );
	
	Angulum = atan2 (Tot[0].Psin,Tot[0].Pcos);   
	Angolo_falso = Angulum*180/PI;
	
	Angulum_1 = atan2 (Tot[0].RPsin,Tot[0].RPcos);
	Angolo_falso_1 = Angulum_1*180/PI;
	
	Angulum_2 = atan2 (Tot[0].APsin,Tot[0].APcos);
	Angolo_falso_2 = Angulum_2*180/PI;


}


void hospitalRobot::Strettoia () {
	int j = 0;	

	for (int i=180; i<Laser_sector; i++){
		
		L[j].Lunghezza = L[i].Dist;
		L[j].Angolo = i;
		j=j+1;
	}
	for (int i=0; i<180; i++){

		L[j].Lunghezza = L[i].Dist;
		L[j].Angolo = i;
		j=j+1;
	}
		
	for(int i=0; i<360-1; i++){
		LatoA = 0;
		LatoB = 0;
		AngoloAB = 0;
		Larghezza = 0;


		if (L[i].Lunghezza+0.1<L[i+1].Lunghezza && L[i].Lunghezza < 3){

			for(int k=i+1; k<360-1; k++){
				if (L[k].Lunghezza<L[i].Lunghezza+0.01 && L[k].Lunghezza>L[i].Lunghezza-0.01){
				 	LatoA = i;
					LatoB = k;
					AngoloAB = abs(LatoB - LatoA);
					Larghezza = 2*L[LatoA].Lunghezza*sin(AngoloAB*RAD/2);
				}
			}
		}

		if (Larghezza<0.6 && (LatoA != 0 || LatoB != 0)){
			
			for (int k = LatoA-1; k > 0 ; k--){
				if (L[k].Dist < L[k+1].Dist){
					LatoA = k;
					k = 0;
				}
			}

			for (int k = LatoB+1; k < 360 ; k++){
				if (L[k].Dist < L[k+1].Dist){
					LatoB = k;
					k = 360;
				}
			}

			for (int i=LatoA; i<LatoB; i++){
				L[L[i].Angolo].Dist = L[LatoA].Lunghezza ;
				
			}
		}

	}
	
}


void hospitalRobot::Correzioneangolo (){

	if (Bot[0].Deg_Theta<0 && G[0].Deg_Theta>0) {
		if (Bot[0].Deg_Theta +180 > G[0].Deg_Theta) {
			G[0].Deg_Theta=G[0].Deg_Theta;
			
		}
		if (Bot[0].Deg_Theta +180 < G[0].Deg_Theta) {
			G[0].Deg_Theta=-360-G[0].Deg_Theta;
			
		}
	}
	if (Bot[0].Deg_Theta>0 && G[0].Deg_Theta <0) {
		if (Bot[0].Deg_Theta -180 < G[0].Deg_Theta) {
			G[0].Deg_Theta=G[0].Deg_Theta;
			
		}

		if (Bot[0].Deg_Theta -180 > G[0].Deg_Theta) {
			G[0].Deg_Theta=360+G[0].Deg_Theta;
			
		}
	}
}


void hospitalRobot::Correzionesettore (){

	if (Bot[0].Theta_z<0 && Bot[0].Theta_w<0 ) { 
		Bot[0].Theta_z=-Bot[0].Theta_z;
		Bot[0].Theta_w=-Bot[0].Theta_w;

	}

	if (Bot[0].Theta_z>0 && Bot[0].Theta_w<0 ) { 
		Bot[0].Theta_z=-Bot[0].Theta_z;
		Bot[0].Theta_w=-Bot[0].Theta_w;
	}

	if (Bot[0].Theta_z>0 && Bot[0].Theta_w>0  || Bot[0].Theta_z<0 && Bot[0].Theta_w>0 ) {
		Bot[0].Theta_z=Bot[0].Theta_z;
		Bot[0].Theta_w=Bot[0].Theta_w;
	}

}

void hospitalRobot::Go_Rotation (){
	
	geometry_msgs::Twist cmd_vel;
	geometry_msgs::Point wheel_vel;	
	geometry_msgs::Point error;	
	
	
	Bot[0].K1 = k_1* (G[0].Pos_x-Bot[0].X_B);
	Bot[0].K2 = k_2* (G[0].Pos_y-Bot[0].Y_B);
	Bot[0].V_dx = Tot[0].Fcos + Bot[0].K1;
	Bot[0].V_dy = Tot[0].Fsin + Bot[0].K2;
 
	Bot[0].Forvel = Bot[0].V_dx*cos (Bot[0].Rad_Theta)+Bot[0].V_dy*sin (Bot[0].Rad_Theta);
	Bot[0].Rotvel = -(Bot[0].V_dx/bi)*sin (Bot[0].Rad_Theta)+(Bot[0].V_dy/bi)*cos (Bot[0].Rad_Theta);
	
	if (Bot[0].Forvel>LIN_VEL){Bot[0].Forvel=LIN_VEL;}
	if (Bot[0].Forvel<-LIN_VEL){Bot[0].Forvel=-LIN_VEL;}	
	if (Bot[0].Rotvel>ANG_VEL){Bot[0].Rotvel=ANG_VEL;}
	if (Bot[0].Rotvel<-ANG_VEL){Bot[0].Rotvel=-ANG_VEL;}

	Bot[0].VelLeft = (Bot[0].Forvel - (Bot[0].Rotvel*DistWheel/2))/RadWheel;
	Bot[0].VelRight = (Bot[0].Forvel + (Bot[0].Rotvel*DistWheel/2))/RadWheel;

	if (Avanti == false){
		ros::Duration(10).sleep();
		Avanti = true; }
	if (Avanti == true && G[0].Dist>0.05){	
	cmd_vel.linear.x = Bot[0].Forvel;
	cmd_vel.angular.z = Bot[0].Rotvel;
	wheel_vel.x = Bot[0].VelLeft;
	wheel_vel.y = Bot[0].VelRight;
	}
	else {
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
	wheel_vel.x = 0;
	wheel_vel.y = 0;
	stop = false ;
	}
	
	Odometry ();

	error.x = Odome[0].Error_x;
	error.y = Odome[0].Error_y;
	error.z = Odome[0].Error_theta;

	_vel_pub.publish( cmd_vel );
	_wheel_pub.publish( wheel_vel );
	_error_pub.publish( error );

}

void hospitalRobot::Center_Laser (){

	int j = 0;
		if ( Grado < 180 ){
			for (int i=Grado+180; i<Laser_sector; i++){
				L[j].Deepth = L[i].Dist;
				L[j].Angle = i;
				j=j+1;
			}
			for (int i=0; i<Grado+180; i++){
				L[j].Deepth = L[i].Dist;
				L[j].Angle = i;
				j=j+1;
			}
		}

		if ( Grado > 180 ){
			for (int i=Grado-180; i<Laser_sector; i++){
				L[j].Deepth = L[i].Dist;
				L[j].Angle = i;
				j=j+1;
			}
			for (int i=0; i<Grado-180; i++){
				L[j].Deepth = L[i].Dist;
				L[j].Angle = i;
				j=j+1;
			}
		}
		
		if ( Grado == 180 ){
			for (int i=0; i<Laser_sector; i++){
				L[j].Deepth = L[i].Dist;
				L[j].Angle = i;
				j=j+1;
			}
		}
}


void hospitalRobot::CambioGoal (){

	if ( Tot[0].FRadice<0.1 && Tot[0].FRadice>-0.1 && Tot[0].PRadice>0 && Val == true ){ 
			
		Center_Laser ();

		for (int i = 180 ; i >2 ;i--){
				
				if (L[i].Deepth+1 < L[i-1].Deepth && L[i-1].Deepth > L[i-2].Deepth && Val == true && L[i-1].Deepth > 1.5 ){
					Angnum_s = L[i-1].Angle;
					Val=false;
					}		
		}
		
		Val = true;
		
		for (int i = 180 ; i < Laser_sector-2 ;i++){
				
				if (L[i].Deepth+0.5 < L[i+1].Deepth && L[i+1].Deepth > L[i+2].Deepth &&Val == true && L[i+1].Deepth > 1.5){
					Angnum_d = L[i+1].Angle;
					Val = false;
					}		
		}
		
		if (L[Angnum_s].Dist > L[Angnum_d].Dist) {Angnum=Angnum_d;}
		else {Angnum=Angnum_s;}
		Val=false;

		Grado = ceil(Angnum+Bot[0].Deg_Theta);
		if (Grado > 180) {Grado = Grado -360;}
		
		PuntoX = (L[Angnum].Dist-0.2)*cos (Grado*RAD) ;
		PuntoY = (L[Angnum].Dist-0.2)*sin (Grado*RAD);
	
		PuntoX = PuntoX + Bot[0].Pos_x ;
		PuntoY = PuntoY + Bot[0].Pos_y ;
		cout<<" Punto X : "<<PuntoX<<"    Punto Y : "<<PuntoY<<  "              con somma"<<endl;
		if (PuntoX > Limite ){PuntoX = Limite-0.25;}
		if (PuntoX < -Limite ){PuntoX = -Limite+0.25;}
		if (PuntoY > Limite ){PuntoY = Limite-0.25;}
		if (PuntoY < -Limite ){PuntoY = -Limite+0.25;}
		cout<<" Punto X : "<<PuntoX<<"    Punto Y : "<<PuntoY<<"           con limiti"<<endl;
		
		G[0].Final_x = G[0].Pos_x;
		G[0].Final_y = G[0].Pos_y;
			
		G[0].Pos_x = PuntoX;
		G[0].Pos_y = PuntoY;


	}
		
	if (G[0].Dist<0.2 && Val == false ){
		G[0].Pos_x = G[0].Final_x;
		G[0].Pos_y = G[0].Final_y;
	}



}

void hospitalRobot::Odometry (){

	Odome[0].Delta_R = Bot[0].VelRight*T_s ;
	Odome[0].Delta_L = Bot[0].VelLeft*T_s ;

	Odome[0].Forvel = (RadWheel/(2*T_s))*(Odome[0].Delta_R+Odome[0].Delta_L);
	Odome[0].Rotvel = (RadWheel/(DistWheel*T_s))*(Odome[0].Delta_R-Odome[0].Delta_L);

	Odome[1].Pos_x = Bot[0].Pos_x + Odome[0].Forvel*T_s*cos(Bot[0].Rad_Theta+(Odome[0].Rotvel*T_s)/2);
	Odome[1].Pos_y = Bot[0].Pos_y + Odome[0].Forvel*T_s*sin(Bot[0].Rad_Theta+(Odome[0].Rotvel*T_s)/2);
	Odome[1].Theta = Bot[0].Rad_Theta + Odome[0].Rotvel*T_s;

 	Odome[0].Error_x = Odome[1].Pos_x - Odome[0].Pos_x; 
	Odome[0].Error_y = Odome[1].Pos_y - Odome[0].Pos_y; 
	Odome[0].Error_theta = Odome[1].Theta - Odome[0].Theta; 
		
	Odome[0].Pos_x = Bot[0].Pos_x;
	Odome[0].Pos_y = Bot[0].Pos_y;
	Odome[0].Theta = Bot[0].Rad_Theta;
}


void hospitalRobot::run(){
	
	boost::thread Total_Potential_t( &hospitalRobot::Total_Potential, this );
	boost::thread Print_t( &hospitalRobot::Print, this );
	boost::thread Go_Rotation_t( &hospitalRobot::Go_Rotation, this );
	ros::Duration(0.2).sleep();
	
}



int main( int argc, char** argv) {
	
   	ros::init(argc, argv, "move_base_client");
	
	hospitalRobot HR;
	HR.Inizialization ();

	while(ros::ok()) {
		HR.run();
		ros::spinOnce();
		
	}
	
return 0;
    	

}

