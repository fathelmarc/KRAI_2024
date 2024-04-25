#ifndef FUZZY_H
#define FUZZY_H
#include <iostream>
#include <cmath>
using namespace std;
class Fuzzy{
    double kecepatan_sangat_lambat(double x){
        if(x >= 2 && x < 5) return 0.2;
        else return 0;
    }
    double kecepatan_lambat(double x){
        if(x >= 5 && x < 10) return 0.5;
        else return 0;
    }
    double kecepatan_sedang(double x){
        if(x >=10 && x < 15) return 1;
        else return 0;
    }
    double kecepatan_tinggi(double x){
        if (x >= 15 && x < 20) return 1.5;
        else return 0;
    }
    double kecepatan_sangat_tinggi(double x){
        if (x >= 20) return 2;
        else return 0;
    }
    double arah_kanan(double x) {
        if (x <= 45) return 0;
        else if (x >= 135) return 1.0;
        else return (x - 4.5) / 9;
    }

    double arah_tengah(double x) {
        if (x <= 45 || x >= 135) return 0;
        else if (x <= 90) return (x - 45) / 45;
        else return (135 - x) / 45;
    }

    double arah_kiri(double x) {
        if (x <= 90) return 1.0;
        else if (x >= 180) return 0;
        else return (18 - x) / 9;
    }
    double highest(double x, float batas_atas, float speed){
        if(x >= batas_atas) return speed;
        else return 0;
        printf("h%f\n",speed);
    }
    double arah(double x, float batas_atas, float batas_bawah , float speed){
        if(x <= batas_atas, x >= batas_bawah) return speed;
        else return 0;
        printf("a%f\n", speed);
    }
    double least(double x , float batas_bawah, float speed){
        if(x <= batas_bawah)return speed;
        else return 0;
        printf("l%f\n", speed);
    }
    double nHighest(double x, float batas_atas, float speed){
        if(x <= batas_atas) return speed;
        else return 0;
        printf("h%f\n",speed);
    }
    double nArah(double x, float batas_atas, float batas_bawah , float speed){
        if(x >= batas_atas, x <= batas_bawah) return speed;
        else return 0;
        printf("a%f\n", speed);
    }
    double nLeast(double x , float batas_bawah, float speed){
        if(x >= batas_bawah)return speed;
        else return 0;
        printf("l%f\n", speed);
    }

    // Implementasikan fungsi kecepatan dan arah fuzzy
    double fuzzy_speed(double distance) {
        double stop = kecepatan_sangat_lambat(distance);
        double very_low= kecepatan_lambat(distance);
        double low = kecepatan_sedang(distance);
        double medium = kecepatan_tinggi(distance);
        double high = kecepatan_sangat_tinggi(distance);
        // Menentukan kecepatan berdasarkan nilai keanggotaan
        double speed = (stop *0 + very_low *0.5 + low * 1 + medium * 1.5 + high * 2); // (stop + very_low + low + medium + high);
        return speed;
    }

    double fuzzy_direction(double angle) {
        double vh = highest(angle, 90,2+1);
        double h = arah(angle, 44, 20,1.5+1);
        double m = arah(angle,20, 10, 1+1 );
        double s = arah(angle, 9, 5, 0.5+1);
        double vs = least(angle, 4, 0+1);

        double nvh = nHighest(angle, -90,-2-1);
        double nh = nArah(angle, -44, -20,-1.5-1);
        double nm = nArah(angle,-20, -10, -1 -1);
        double ns = nArah(angle, -9, -5, -0.5-1);
        double nvs = nLeast(angle, -4, -0-1);
        // Menentukan arah berdasarkan nilai keanggotaan
        // double direction;
        printf("angle:%f\n", angle);
        printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",vh,h,m,s,vs,nvh,nh,nm,ns,nvs);
        float direction = (nvs*0 + ns*0.5  + nm*1 + nh*1.5 + nvh*2);
        if (angle > 0){
            direction = (vs*0 + s*0.5  + m*1  + h*1.5 + vh*2 );
        }
        return direction;
    }
    public:
    void target(float target_x, float target_y, float target_orientation, float robot_x, float robot_y, float robot_orientation) {
        // Hitung jarak dan sudut antara target dan robot
        double distance = std::sqrt(std::pow(target_x - robot_x, 2) + std::pow(target_y - robot_y, 2));
        double angle = std::atan2(target_y - robot_y, target_x - robot_x);
        
        // Normalisasi sudut ke rentang -180 hingga 180
        // if (angle > 180)
        //     angle -= 360;
        // else if (angle < -180)
        //     angle += 360;
        
        // Fuzzy logic untuk kecepatan dan arah
        double speed = fuzzy_speed(distance);
        double errorT = target_orientation - robot_orientation;
        double direction = fuzzy_direction(errorT);
        
        // Hitung perubahan posisi berdasarkan kecepatan dan arah
        double delta_x = speed * cos(angle);
        double delta_y = speed * sin(angle);
        double delta_h = direction;
        inKinematic(delta_x, delta_y, delta_h, passX);
        printf("robot :%f,%f,%f\n",delta_x,delta_y,direction);
    }
};
#endif