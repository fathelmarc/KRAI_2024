#include <iostream>
#include <cmath>

// Definisikan fungsi keanggotaan untuk variabel kecepatan dan arah
double kecepatan_rendah(double x) {
    if (x <= 20) return 1.0;
    else if (x <= 40) return (40 - x) / 20;
    else return 0.0;
}

double kecepatan_sedang(double x) {
    if (x <= 20 || x >= 60) return 0.0;
    else if (x <= 40) return (x - 20) / 20;
    else return (60 - x) / 20;
}

double kecepatan_tinggi(double x) {
    if (x >= 60) return 1.0;
    else if (x >= 40) return (x - 40) / 20;
    else return 0.0;
}

double arah_kanan(double x) {
    if (x <= 45) return 0.0;
    else if (x >= 135) return 1.0;
    else return (x - 45) / 90;
}

double arah_tengah(double x) {
    if (x <= 45 || x >= 135) return 0.0;
    else if (x <= 90) return (x - 45) / 45;
    else return (135 - x) / 45;
}

double arah_kiri(double x) {
    if (x <= 90) return 1.0;
    else if (x >= 180) return 0.0;
    else return (180 - x) / 90;
}

// Implementasikan fungsi kecepatan dan arah fuzzy
double fuzzy_speed(double distance) {
    double low = kecepatan_rendah(distance);
    double medium = kecepatan_sedang(distance);
    double high = kecepatan_tinggi(distance);
    // Menentukan kecepatan berdasarkan nilai keanggotaan
    double speed = (low * 10 + medium * 30 + high * 50) / (low + medium + high);
    return speed;
}

double fuzzy_direction(double angle) {
    double left = arah_kiri(angle);
    double center = arah_tengah(angle);
    double right = arah_kanan(angle);
    // Menentukan arah berdasarkan nilai keanggotaan
    double direction = (left * -1 + center * 0 + right * 1) / (left + center + right);
    return direction;
}

void fuzzy_target(float target_x, float target_y, float target_orientation, float robot_x, float robot_y, float robot_orientation) {
    // Hitung jarak dan sudut antara target dan robot
    double distance = std::sqrt(std::pow(target_x - robot_x, 2) + std::pow(target_y - robot_y, 2));
    double angle = std::atan2(target_y - robot_y, target_x - robot_x) * (180.0 / M_PI) - robot_orientation;
    
    // Normalisasi sudut ke rentang -180 hingga 180
    if (angle > 180)
        angle -= 360;
    else if (angle < -180)
        angle += 360;
    
    // Fuzzy logic untuk kecepatan dan arah
    double speed = fuzzy_speed(distance);
    double direction = fuzzy_direction(angle);
    
    // Hitung perubahan posisi berdasarkan kecepatan dan arah
    double delta_x = speed * cos((robot_orientation + angle) * M_PI / 180.0);
    double delta_y = speed * sin((robot_orientation + angle) * M_PI / 180.0);
    
    // Update posisi robot
    robot_x += delta_x;
    robot_y += delta_y;
    
    // Update orientasi robot
    robot_orientation += direction * 45; // Misalnya, 45 derajat adalah maksimum perubahan arah
    
    // Output posisi dan orientasi robot setelah pergerakan
    std::cout << "Posisi robot setelah pergerakan:" << std::endl;
    std::cout << "X: " << robot_x << std::endl;
    std::cout << "Y: " << robot_y << std::endl;
    std::cout << "Orientasi: " << robot_orientation << std::endl;
}

int main() {
    
    fuzzy_target(0,0,0,0,0,0);
    
    return 0;
}
