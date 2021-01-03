#include <stdio.h>
#include <iostream>
#include <Eigen/Dense> 
#include <math.h>
#define pi 3.1415926535898

// Eigen::Matrix<double, 6, 1> Cal_global_vel2motor_vel (double vx , double vy , double vz , double v_roll , double v_pitch , double v_yaw , Eigen::Matrix<double, 6, 1> motor_angle);
Eigen::Matrix<double, 6, 1> Cal_global_vel2motor_vel (Eigen::Matrix<double , 6, 1> global_vel, Eigen::Matrix<double, 6, 1> motor_angle);
Eigen::Matrix<double, 6, 6> J_solve(Eigen::Matrix<double, 6, 1> motor_angle);
Eigen::Matrix<double, 4, 4> T_solve (double a,double alpha,double d,double theta );
Eigen::Matrix<double, 3, 1> rodriguez (Eigen::Matrix<double, 3, 3> pose,Eigen::Matrix<double, 3, 1> k);
Eigen::Matrix<double, 3, 1> tr2RPY(Eigen::Matrix<double, 3, 3> pose);

int main()
{
	Eigen::Matrix<double, 6, 1> b;
	b << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<double, 6, 1> global_vec;
	global_vec << 1, 1, 1, 1, 1, 1;
	Eigen::Matrix<double, 6, 1> a = Cal_global_vel2motor_vel(global_vec, b);
	// for (int i=0;i<6;i++){
	// 	printf("%f \n",a[i]);
	// }
	std::cout << a << std::endl;
}


Eigen::Matrix<double, 6, 1> Cal_global_vel2motor_vel (Eigen::Matrix<double , 6, 1> global_vel, Eigen::Matrix<double, 6, 1> motor_angle){
	Eigen::Matrix<double, 6, 6> J;
	J = J_solve(motor_angle);
	std::cout << "J:" << J << std::endl;
	Eigen::Matrix<double, 6, 1> motor_vel;
	Eigen::Matrix<double, 6, 1> motor_vel_matrix;
	// Eigen::Matrix<double, 6, 1> global_vel;
	// global_vel(0,0)=vx;
	// global_vel(1,0)=vy;
	// global_vel(2,0)=vz;
	// global_vel(3,0)=v_roll;
	// global_vel(4,0)=v_pitch;
	// global_vel(5,0)=v_yaw;
	motor_vel_matrix = J.inverse()*global_vel ;
	for(int i=0;i<6;i++){
		motor_vel[i]=motor_vel_matrix(i,0);
	}
	return motor_vel;
}

Eigen::Matrix<double, 6, 6> J_solve(Eigen::Matrix<double, 6, 1> motor_angle){
	double angle[6];
	double init_angle[6]={90,90,0,180,90,0};
	for (int i=0;i<6;i++){
		angle[i]=motor_angle[i]+init_angle[i];
		while (angle[i]>180)
            angle[i] = angle[i] - 360;
        while (angle[i]<-180)
            angle[i] = angle[i] + 360;
        angle[i] = angle[i]*pi/180.0;    
	}
	
	Eigen::Matrix<double, 4, 4>T01 = T_solve( 0, 90.0/180.0*pi, 253.0, angle[0] );
	Eigen::Matrix<double, 4, 4>T12 = T_solve( 180, 0.0/180.0*pi, 0.0, angle[1] );
	Eigen::Matrix<double, 4, 4>T23 = T_solve( 27, 90.0/180.0*pi, 0.0, angle[2] );
	Eigen::Matrix<double, 4, 4>T34 = T_solve( 0, 90.0/180.0*pi, 182.96, angle[3] );
	Eigen::Matrix<double, 4, 4>T45 = T_solve( 0, 90.0/180.0*pi, 0.0, angle[4] );
	Eigen::Matrix<double, 4, 4>T56 = T_solve( 0, 0.0/180.0*pi, 61.85, angle[5] );
	
	Eigen::Matrix<double, 4, 4>T02 = T01*T12;
	Eigen::Matrix<double, 4, 4>T03 = T01*T12*T23;
	Eigen::Matrix<double, 4, 4>T04 = T01*T12*T23*T34;
	Eigen::Matrix<double, 4, 4>T05 = T01*T12*T23*T34*T45;
	Eigen::Matrix<double, 4, 4>T06 = T01*T12*T23*T34*T45*T56;
//	std::cout << "T:" << T34 << std::endl;	
		
	Eigen::Matrix<double, 3, 1> p0;
	Eigen::Matrix<double, 3, 1> p1;
	Eigen::Matrix<double, 3, 1> p2;
	Eigen::Matrix<double, 3, 1> p3;
	Eigen::Matrix<double, 3, 1> p4;
	Eigen::Matrix<double, 3, 1> p5;
	Eigen::Matrix<double, 3, 1> p6;
	
	p0 << 0,0,0;
	for (int i=0;i<3;i++){
		p1(i,0) = T01(i,3);
		p2(i,0) = T02(i,3);
		p3(i,0) = T03(i,3);
		p4(i,0) = T04(i,3);
		p5(i,0) = T05(i,3);
		p6(i,0) = T06(i,3);
	}
	
	
	Eigen::Matrix<double, 3, 1> z0;
	Eigen::Matrix<double, 3, 1> z1;
	Eigen::Matrix<double, 3, 1> z2;
	Eigen::Matrix<double, 3, 1> z3;
	Eigen::Matrix<double, 3, 1> z4;
	Eigen::Matrix<double, 3, 1> z5;
	z0 << 0,0,1;
	for (int i=0;i<3;i++){
		z1(i,0) = T01(i,2);
		z2(i,0) = T02(i,2);
		z3(i,0) = T03(i,2);
		z4(i,0) = T04(i,2);
		z5(i,0) = T05(i,2);
	}
	
	Eigen::Matrix<double, 6, 6> J;
	J << z0.cross(  (p6-p0) ), z1.cross( (p6-p1) ), z2.cross( (p6-p2) ), z3.cross( (p6-p3) ), z4.cross( (p6-p4) ), z5.cross( (p6-p5) ),
         z0, z1, z2, z3, z4, z5;
//    std::cout << "J:" << J << std::endl;
    for(int i=0;i<6;i++){
    	for(int j=0;j<6;j++){
    		J(i,j)=J(i,j)*pi/180.0;
    	}
    }
    
    Eigen::Matrix<double, 3, 3> R;
    for (int i=0;i<3;i++){
	    for (int j=0;j<3;j++){
	    	R(i,j)=T06(i,j);
	    }
	}
	Eigen::Matrix<double, 3, 1> Temp_J;
    for (int i=0;i<6;i++){
    	for(int j=3;j<6;j++){
    		Temp_J(j-3,0)=J(j,i);
    	}
    	Temp_J=rodriguez( R, Temp_J );
    	for(int j=3;j<6;j++){
    		J(j,i)=Temp_J(j-3,0);
    	}
    }    
    
    return J;
}

Eigen::Matrix<double, 4, 4> T_solve (double a,double alpha,double d,double theta ){
	Eigen::Matrix<double, 4, 4> T;
	T << cos(theta),-cos(alpha)*sin(theta),sin(alpha)*sin(theta),a*cos(theta),
	     sin(theta),cos(alpha)*cos(theta),-sin(alpha)*cos(theta),a*sin(theta),
	     0,sin(alpha),cos(alpha),d,
         0,0,0,1;
    return T;
}

Eigen::Matrix<double, 3, 1> rodriguez (Eigen::Matrix<double, 3, 3> pose,Eigen::Matrix<double, 3, 1> k){
	double delta = 0.01;
	Eigen::Matrix<double, 3, 1> delta_m;
	delta_m << delta,delta,delta;
	Eigen::Matrix<double, 3, 3> K;
	K <<  0      , -k(2,0) , k(1,0) ,
          k(2,0) , 0       , -k(0,0),
          -k(1,0), k(0,0)  , 0      ;
    Eigen::Matrix<double, 3, 3> C,R,d_pose;
    Eigen::Matrix<double, 3, 1> d_RPY,RPY;
    C.setIdentity(3,3);
    R = C + (1-cos(delta))*K*K + sin(delta)*K;
    
    d_pose = R*pose;
    d_RPY = tr2RPY(d_pose)-tr2RPY(pose);
    
    for (int i=0;i<3;i++){
    	while (d_RPY(i,0)<-pi)
    	    d_RPY(i,0) = d_RPY(i,0)+2*pi;
    	while (d_RPY(i,0)>pi)
    	    d_RPY(i,0) = d_RPY(i,0)-2*pi;
    }
    RPY=d_RPY.cwiseQuotient(delta_m);
    return RPY;
}

Eigen::Matrix<double, 3, 1> tr2RPY (Eigen::Matrix<double, 3, 3> pose){
	double R = atan2( pose(2,1), pose(2,2) );
	double Y = atan2( pose(1,0), pose(0,0) );
	double P = atan2( -pose(2,0), sqrt( pose(2,1)*pose(2,1)+ pose(2,2)*pose(2,2) ));
	Eigen::Matrix<double, 3, 1> RPY;
	RPY << R,P,Y;
	return RPY;
}








