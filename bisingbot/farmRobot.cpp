#include "iostream"
#include "chrono"
#include "thread"
#include "ethernet.h"

#define LeftHatX 0
#define LeftHatY 1
#define RightHatX 3
#define RightHatY 4
#define SILANG 0
#define CIRCLE 1
#define TRIANGLE 2
#define SQUARE 3
#define L1 4
#define R1 5
#define L2 2
#define R2 5
#define butY 7 
#define butX 6
#define UP 11
#define DOWN -11
int caseBot;
int prevcase;
int manual;
int tanam, ambil;
static int speed;
float speedChange[3] = {1,2,5};

clock_t startcount;

//1 = 69 | new 1 = 67
//2 = 68 | new 2 = 66
//3 = 67 | new 3 = 69
//4 = 66 | new 4 = 68
int Xtar,Ytar,Xcurr,Ycurr;

void forKinematic(){
    roda1.terimaData(sockfd);
    roda2.terimaData(sockfd);
    roda3.terimaData(sockfd);
    roda4.terimaData(sockfd);
    double m1= ((roda1.en1-roda1.enprev1)*kRoda/PPR)/4;
    double m2= ((roda2.en2-roda2.enprev2)*kRoda/PPR)/4;
    double m3= ((roda3.en3-roda3.enprev3)*kRoda/PPR)/4;
    double m4= ((roda4.en4-roda4.enprev4)*kRoda/PPR)/4;

    roda1.enprev1 = roda1.en1;
    roda2.enprev2 = roda2.en2;
    roda3.enprev3 = roda3.en3;
    roda4.enprev4 = roda4.en4;

    float fx = sin(toRad(angleM1))*m1 + sin(toRad(angleM2))*m2 + sin(toRad(angleM3))*m3 + sin(toRad(angleM4))*m4;
    float fy = cos(toRad(angleM1))*m1 + cos(toRad(angleM2))*m2 + cos(toRad(angleM3))*m3 + cos(toRad(angleM4))*m4;
    passX += fx;
    passY += fy;
    
    // printf("\nm1 =%f\nm2 =%f\nm3 =%f\nm4 =%f\n",roda1.en1,roda2.en2,roda3.en3,roda4.en4);
}

void inKinematic(float vx, float vy, float vt, float currT){
    v1 = ((sin(toRad(angleM1 + currT)) * vx) + (cos(toRad(angleM1 + currT)) * vy) + (vt * R_robot))/R_roda;
    v2 = ((sin(toRad(angleM2 + currT)) * vx) + (cos(toRad(angleM2 + currT)) * vy) + (vt * R_robot))/R_roda;
    v3 = ((sin(toRad(angleM3 + currT)) * vx) + (cos(toRad(angleM3 + currT)) * vy) + (vt * R_robot))/R_roda;
    v4 = ((sin(toRad(angleM4 + currT)) * vx) + (cos(toRad(angleM4 + currT)) * vy) + (vt * R_robot))/R_roda;

    roda1.kirimData(sockfd, std::to_string(v1));
    roda2.kirimData(sockfd, std::to_string(v2));
    roda3.kirimData(sockfd, std::to_string(v3));
    roda4.kirimData(sockfd, std::to_string(v4));
    atas.kirimData(sockfd, std::to_string(hold));
    // printf("%f,%f,%f\n",vx,vy,vt);
    // printf("\nv1=%f\nv2=%f\nv3=%f\nv4=%f\n",v1,v2,v3,v4); 
}

const char* fileName = "/dev/input/js99"; // Anggap kita hanya memiliki satu joystick dan itu adalah js0
Joystick joy = openJoystick(fileName);
void inCaseAuto(){
    forKinematic();
    atas.terimaData(sockfd);
    passT = atas.meg;
    readJoystickInput(&joy);
    static int axisCont = 0;
    static int prevAxisValue = 0;
    if (joy.axisStat[7] > prevAxisValue && !axisCont) {
        speed--;
        axisCont =1;
    }
    if (joy.axisStat[7] < prevAxisValue && !axisCont) {
        speed++;
        axisCont = 1;
    }else if(joy.axisStat[7] == 0){
        axisCont = 0;
    }else if(speed >= 4){
        speed = 0;
    }else if(speed <= -1){
        speed = 3;
    }
    prevAxisValue = joy.axisStat[7];
    if (joy.bStat[SILANG] && prevcase == 0) {
        caseBot = 1;
        prevcase = caseBot;
    }
    if (joy.bStat[TRIANGLE] && prevcase == 1) {
        caseBot = 2;
        prevcase = caseBot;
    }
    if(joy.bStat[SILANG] && prevcase == 2){
        caseBot = 3;
        prevcase = caseBot;
    }
    if(joy.bStat[TRIANGLE] && prevcase == 3){
        caseBot = 4;
        prevcase = caseBot;
    }
    if(joy.bStat[SILANG] && prevcase == 4){
        caseBot = 5;
        prevcase = caseBot;
    }
    if(joy.bStat[TRIANGLE] && prevcase == 5){
        caseBot = 6;
        prevcase = caseBot;
    }if (joy.bStat[SILANG] && prevcase == 6) {
        caseBot = 7;
        prevcase = caseBot;
    }
    if (joy.bStat[TRIANGLE] && prevcase == 7) {
        caseBot = 8;
        prevcase = caseBot;
    }
    if(joy.bStat[SILANG] && prevcase == 8){
        caseBot = 9;
        prevcase = caseBot;
    }
    if(joy.bStat[TRIANGLE] && prevcase == 9){
        caseBot = 10;
        prevcase = caseBot;
    }
    if(joy.bStat[SILANG] && prevcase == 10){
        caseBot = 11;
        prevcase = caseBot;
    }
    if(joy.bStat[TRIANGLE] && prevcase == 11){
        caseBot = 12;
        prevcase = caseBot;
    }else if(joy.bStat[10]){
        manual = 0;
        caseBot = 0;
    }else if(joy.bStat[9]){
        prevcase = 0;
    }
    // printf("casebot = %d\nprevcase = %d\nspeed = %d\n", caseBot,prevcase,speed);
}

float kp =0.05;//0.4;//mulus(0.13);//3.06;//1.08;//1.8;//0.51;//0.85;//0.36
float ki =0.00005;//.03;//0;//.03;//.013;//mulus(0);//.017;//0.0372;//0;//0.102;//0;
float kd =0.0035;//.00396;//0.00396;//.00186;//mulus(0);//.01317;//6.83;//0;//0.6375;//0;

float kpt =0.017;//0.06;
float kit =0.0;//.005;//0.0062;
float kdt =0;//.02;//0.0016;

void pidY(float targetY){
    forKinematic();
    double deltaTime = calculateDeltaTime();

    errY = targetY - passY;
    float errInY = errInY + errY * deltaTime;

    float errDerY = (errY - errprevY)/deltaTime;
    errprevY = errY;

    float uy = kp * errY + ki * errInY + kd * errDerY;
    ly = fmax(-MAXSPEED,fmin(uy,MAXSPEED));
}
void pidX(float targetX){
    forKinematic();
    double deltaTime = calculateDeltaTime();

    errX = targetX - passX;

    float errInX =+ errX * deltaTime;

    float errDerX = (errX - errprevX)/deltaTime;
    errprevX = errX;

    float ux = kp * errX + ki * errInX + kd * errDerX;
    lx = fmax(-MAXSPEED,fmin(ux,MAXSPEED));
}
void pidT(float targetT){
    forKinematic();
    atas.terimaData(sockfd);
    double deltaTime = calculateDeltaTime();
    passT = atas.meg;
    errT = targetT - passT;
    if (errT > 180 || errT < -180){
        errT = passT - targetT;
    }
    float errInT = errInT + errT * deltaTime;
    
    float errDerT = (errT - errprevT)/deltaTime;
    errprevT = errT;
    
    float ut = kpt * errT + kit * errInT + kdt * errDerT;
    lt = fmax(-MAXSPEEDT,fmin(ut,MAXSPEEDT));
}
void setPos(float targetX, float targetY, float targetT){
    pidY(targetY);
    pidT(targetT);
    pidX(targetX);
    inKinematic(lx,ly,lt,passT);
    // printf("%f,%f,%f",passY,ky);
    // printf("errorX = %f\nerrorY = %f\n",errX,errY);
    printf("%f,%f,%d\n",passX,passY,passT);
    // printf("%f,%f,%f",lx,ly,lt);
}
int main(){
    mkIkat();
    if (!joy.connected) {
        printf("joy tidak terhubung\n");
        return 1;
    }
    printf("start robot");
    while(1){
        inCaseAuto();
        // printf("%f,%f,%d\n",passX,passY,passT);
        if(caseBot ==0 && joy.axisStat[LeftHatX] > 1 && joy.axisStat[LeftHatX] < 32769 ||joy.axisStat[LeftHatX] > -32769 && joy.axisStat[LeftHatX] < -1 ||joy.axisStat[LeftHatY] > 1 && joy.axisStat[LeftHatY] < 32769 ||joy.axisStat[LeftHatY] > -32769 && joy.axisStat[LeftHatY] < -1 ||joy.axisStat[RightHatX] > 1 && joy.axisStat[RightHatX] < 32769 ||joy.axisStat[RightHatX] < -1 && joy.axisStat[RightHatX] > -32769){
            float hx = joy.axisStat[LeftHatX] *  speedChange[speed]/1000000;
            float hy = joy.axisStat[LeftHatY] * -speedChange[speed]/1000000;
            float ht = joy.axisStat[RightHatX]*  speedChange[speed]/1000000;
            inKinematic(hx, hy, ht,passT);
            // printf("%f,%f,%f\n",hx,hy,ht);
            // printf("currSpeed = %f\n", speedChange[speed]);
            // printf("%f,%f,%f,%f\n",v1,v2,v3,v4);
        }
        else if(joy.bStat[L1]){//l1
            for(int i =0; i<1000; i++){
                // printf("L1");
                atas.kirimData(sockfd, std::to_string(L1));
            }
        }		
		else if(joy.bStat[R1]){//r1
            for (int i = 0;i<1000;i++){
                // printf("R1");
                atas.kirimData(sockfd, std::to_string(R1));
            }
        }
        else if(joy.bStat[8]){
            setPos(0,0,0);
        }

        //position
        else if(caseBot == 1){
            atas.kirimData(sockfd, std::to_string(R1));
           while(passY != 5.0 && passX > -1 && passX < 1){
                setPos(0,5.0,0);
                // ROS_INFO("maju");
                if(errY < 0.5){
                    break;
                }
            }while(passX != 13.0 && passY > 4 && passY < 6){
                setPos(13.0,5.0,0);
                // ROS_INFO("kanan");
                if(errX < 0.5){
                    break;
                }
            }
        }else if(caseBot == 2){
            atas.kirimData(sockfd, std::to_string(L1));
            setPos(17.1,23.2,0);
        }else if(caseBot == 3){
            atas.kirimData(sockfd, std::to_string(R1));
            setPos(17.6, 5, 0);//x = 15.3
        }else if(caseBot == 4){
            atas.kirimData(sockfd, std::to_string(L1));
            setPos(17.3, 18.4, 0);
        }else if(caseBot == 5){
            atas.kirimData(sockfd, std::to_string(R1));
            setPos(27.6, 5, 0);//x=23.4
        }else if(caseBot == 6){
            atas.kirimData(sockfd, std::to_string(L1));
            setPos(30.3, 23.2,0);
        }

        //gripper
        else if(joy.bStat[SQUARE]){//petak
            for (int i = 0;i<3000;i++){
                // printf("petak");
                atas.kirimData(sockfd, std::to_string(SQUARE));
            }
        }else if(joy.bStat[CIRCLE]){
            for (int i = 0;i<3000;i++){
                // printf("bulat");
                atas.kirimData(sockfd, std::to_string(CIRCLE));
			}
        }

        else{
            atas.kirimData(sockfd,std::to_string(hold));
            inKinematic(0,0,0,passT);
        }
        fflush(stdout);
        usleep(16000);

        // std::this_thread::sleep_for(std::chrono::milliseconds(16)); 
    }
    close(sockfd);
    return 0;
}