#ifndef CONTROL_H
#define CONTROL_H
#include "motion.h"
#include "std_msgs/Float32MultiArray.h"


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <set>

using namespace std;


void addData(int kasus, float x, float y, float h) {
    ofstream file;
    ifstream readFile;
    stringstream filename;
    set<string> existingData;
    string line;

    if (kasus > 0 && kasus <= 9 && lantai2 == 0) {
        if(kasus == 1){
            filename << "/home/m/catR1/src/movement/src/data_cel_bluezones/ambil1.txt";
        }else if(kasus == 2){
            filename << "/home/m/catR1/src/movement/src/data_cel_bluezones/tanam1.txt";
        }else if(kasus == 3){
            filename << "/home/m/catR1/src/movement/src/data_cel_bluezones/tanam2.txt";
        }
        
        else if(kasus == 4){
            filename << "/home/m/catR1/src/movement/src/data_cel_bluezones/ambil2.txt";
        }else if(kasus == 5){
            filename << "/home/m/catR1/src/movement/src/data_cel_bluezones/tanam3.txt";
        }else if(kasus == 6){
            filename << "/home/m/catR1/src/movement/src/data_cel_bluezones/tanam4.txt";
        }
        
        else if(kasus == 7){
            filename << "/home/m/catR1/src/movement/src/data_cel_bluezones/ambil3.txt";
        }else if(kasus == 8){
            filename << "/home/m/catR1/src/movement/src/data_cel_bluezones/tanam5.txt";
        }else if(kasus == 9){
            filename << "/home/m/catR1/src/movement/src/data_cel_bluezones/tanam6.txt";
        }
        // Open the file for reading
        readFile.open(filename.str());
        if (!readFile.is_open()) {
            cerr << "Unable to open file for reading: " << filename.str() << endl;
            return;
        }
        
        // Read existing data from file
        while (getline(readFile, line)) {
            existingData.insert(line);
        }
        readFile.close();

        // Prepare the new data string
        string newData = to_string(x) + "," + to_string(y) + "," + to_string(h);

        // Check if the new data is already in the set
        if (existingData.find(newData) == existingData.end()) {
            // Open the file for appending
            file.open(filename.str(), ios::app);
            if (!file.is_open()) {
                cerr << "Unable to open file for writing: " << filename.str() << endl;
                return;
            }
            file << newData << endl;
            file.close();
        }
    }
}

// Function to process each line and accumulate the sums for each column
void processLine(const string& line, float& sumX, float& sumY, int& sumT, int& count) {
    stringstream lineStream(line);
    string cell;
    vector<float> values;

    // Split the line by commas and convert to float
    while (getline(lineStream, cell, ',')) {
        values.push_back(stof(cell));
    }

    // Accumulate the sums for each column
    if (values.size() >= 3) {
        sumX += values[0];
        sumY += values[1];
        sumT += static_cast<int>(values[2]);
        count++;
    }
}


// Function to compute and return the averages based on type
vector<double> computeAverages(const string& filename) {
    ifstream inputFile(filename);
    vector<double> averages(3, -1.0);  // Initialize with -1.0 to indicate error

    if (!inputFile) {
        cerr << "Error opening file: " << filename << endl;
        return averages;
    }

    float sumX = 0.0;
    float sumY = 0.0;
    int sumT = 0;
    int count = 0;
    string line;

    while (getline(inputFile, line)) {
        processLine(line, sumX, sumY, sumT, count);
    }

    inputFile.close();

    if (count > 0) {
        averages[0] = sumX / count;
        averages[1] = sumY / count;
        averages[2] = static_cast<float>(sumT) / count;
    } else {
        cerr << "No data to process in file: " << filename << endl;
    }

    return averages;
}

#endif  
