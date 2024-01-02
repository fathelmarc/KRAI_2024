#include "iostream"
#include "stdio.h"
#include <thread>
#include "ethernet.cpp"
using namespace std;

#define R_roda 0.05
#define R_robot 0.16149
#define PPR 200
#define MAXSPEED 0.25
const int angleM1 = 45;
const int angleM2 = 135;
const int angleM3 = 225;
const int angleM4 = 315;
const int angleM[4] = {45,135,225,315};
float v[4];
double m1,m2,m3,m4;
volatile float prevErr_x,prevErr_y,prevErr_t;
float v1,v2,v3,v4,lx,ly,lt,deltaTime,errprev,kx,ky,kt;
float kecM1;
float kecM2;
float kecM3;
float kecM4,passT;
volatile long prevTime;
int passX,passY;
enum State {PID_STATE_1, PID_STATE_2};
State currentState = PID_STATE_1;

void time(){
deltaTime = clock() - prevTime;
prevTime = clock();
}

char buffer[MAXLINE];
int sockfd,sizeReceive;
struct sockaddr_in serv_addr,cli_addr;
socklen_t len;

Device roda1("192.168.0.69",5555);
Device roda2("192.168.0.68",5555);
Device roda3("192.168.0.67",5555);
Device roda4("192.168.0.66",5555);
Device imu("192.168.0.70",5555);

void ForKinematic(){
	roda1.terimaData(sockfd);
	roda2.terimaData(sockfd);
	roda3.terimaData(sockfd);
	roda4.terimaData(sockfd);
	m1= (roda1.en1 * M_PI * 0.1) / PPR ;
	m2= (roda2.en2 * M_PI * 0.1) / PPR ;
	m3= (roda3.en3 * M_PI * 0.1) / PPR ;
	m4= (roda4.en4 * M_PI * 0.1) / PPR ;
    float var = sqrt(2);
    float fx = ((var * m1) + (var * m2) + (-var * m3) + (-var * m4)) / 4;
    float fy = ((var * m1) + (-var * m2) + (-var * m3) + (var * m4)) / 4;
	float ft = (m1+m2+m3+m4)/R_robot*4;
    passX = fx;
    passY = fy;
	passT = ft;
	// printf("%f,%f,%f,%f\n",roda1.en1,roda2.en2,roda3.en3,roda4.en4);
}
void InKinematic(float vx, float vy, float vt){
	v1 = (sin(toRad(angleM1)) * vx + cos(toRad(angleM1)) * vy + R_robot * vt)/R_roda;
	v2 = (sin(toRad(angleM2)) * vx + cos(toRad(angleM2)) * vy + R_robot * vt)/R_roda;
	v3 = (sin(toRad(angleM3)) * vx + cos(toRad(angleM3)) * vy + R_robot * vt)/R_roda;
	v4 = (sin(toRad(angleM4)) * vx + cos(toRad(angleM4)) * vy + R_robot * vt)/R_roda;
	roda1.kirimData(sockfd, std::to_string(v1));
	roda2.kirimData(sockfd, std::to_string(v2));
	roda3.kirimData(sockfd, std::to_string(v3));
	roda4.kirimData(sockfd, std::to_string(v4));
	// printf("%f,%f,%f,%f\n", v1Filt,v2Filt,v3Filt,v4Filt);
	printf("%f,%f,%f,%f\n", m1,m2,m3,m4);
}

void PIDx(float target){
	time();
	float kp =0;
	float ki =0;
	float kd =0;
	float err = (target - passX)*10*M_PI/PPR;
	float errIn = errIn + err * deltaTime;
	float errDer = (err - errprev)/deltaTime;
	errprev = err;
	float ux = kp * err + ki * errIn + kd * errDer;
	kalmanX(ux);
	kx = fmaxf(-MAXSPEED,fminf(lx,MAXSPEED));
}
void PIDy(float target){
	time();
	float kp =0.05;
	float ki =0;
	float kd =0;
	float err = (target - passY)*10*M_PI/PPR;
	float errIn = errIn + err * deltaTime;
	float errDer = (err - errprev)/deltaTime;
	errprev = err;
	float uy = kp * err + ki * errIn + kd * errDer;
	kalmanY(uy);
	ky = fmaxf(-MAXSPEED,fminf(ly,MAXSPEED));
}
void PIDt(float target){
	time();
	float kp =0;
	float ki =0;
	float kd =0;
	imu.terimaData(sockfd);
	imu.imuDir = passT;
	float err = (passT - target)*10*M_PI/PPR;
	if (err > 180 || err <180){
		err = (target - passT)*10*M_PI/PPR;
	}
	float errIn = errIn + err * deltaTime;
	float errDer = (err - errprev)/deltaTime;
	errprev = err;
	float ut = kp * err + ki * errIn + kd * errDer;
	kalmanT(ut);
	kt = fmaxf(-MAXSPEED,fminf(lt,MAXSPEED));
}

void setPos(int targetX, int targetY, int targetT){
	ForKinematic();
	std::thread x(PIDx,targetX);
	std::thread y(PIDy,targetY);
	std::thread t(PIDt,targetT);
	x.join();
	y.join();
	t.join();
    InKinematic(lx, ly, lt);
	printf("%d\n%f\n ",targetY,m4);
}
// Driver code 
int main() {
	// buatkan socket untuk deskripsi file
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
		perror("socket creation failed"); 
		exit(EXIT_FAILURE); 
	} 
	memset(&serv_addr, 0, sizeof(serv_addr));
	// info server semacam addressnya
	serv_addr.sin_family = AF_INET; //IPv4
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(PORT); 
	//ikat socket dengan servernya
	if ( bind(sockfd, (const struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0 ){ 
		perror("bind failed"); 
		exit(EXIT_FAILURE); 
	}

	while (1) {
		setPos(0,10000,0);
		// std::cout<<roda4.en4<<endl;
        // while(passY >= 0 && passY <= 510){
		// 	PID(0,300,0);
		// 	printf("Y");
		// 	if(passY > 300){
		// 		break;
		// 	 }
			
		// }while(passY >= 490 || passY <= 510 && passX >= -5 || passX <= 530){
		// 	PID(500,300,0);
		// 	printf("X");
		// 	if(passX > 500){
		// 		break;
		// 		printf("done");
		// 	}	
		// }while(passY >= 490 || passY <= 510 && passX >= 490 || passX <= 530){
		// 	PID(500,0,0);
		// 	if(passY <= 5){
		// 		break;
		// 	}
		// }while(passY >= 490 || passY <= 510 && passX >= -5 || passX <= 10){
		// 	PID(0,0,0);
		// 	printf("X");
		// 	if(passX > 5){
		// 		break;
		// 		printf("done");
		// 	}	
		// }
	}

	close(sockfd); 
	return 0; 
}
