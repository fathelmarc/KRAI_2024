#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>  // For read, STDIN_FILENO
#include <termios.h> // For tcgetattr, tcsetattr

using namespace std;

// Function to compute the averages of x, y, and t from the file
bool computeAverages(const string& filename, float& avgX, float& avgY, int& avgT) {
    ifstream inputFile(filename);

    if (!inputFile) {
        cerr << "Error opening file: " << filename << endl;
        return false; // Indicate failure to open the file
    }

    float sumX = 0.0;
    float sumY = 0.0;
    int sumT = 0;
    int count = 0;
    string line;

    while (getline(inputFile, line)) {
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

    inputFile.close();

    if (count > 0) {
        avgX = sumX / count;
        avgY = sumY / count;
        avgT = sumT / count; // Integer division
        return true; // Indicate success
    } else {
        cerr << "No data to process." << endl;
        return false; // Indicate failure due to no data
    }
}

// Example function to set terminal to raw mode (for custom key handling)
void setRawMode(struct termios& oldt) {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt); // Save current terminal settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Apply new terminal settings
}

// Example function to restore terminal settings
void resetTerminalMode(const struct termios& oldt) {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore old terminal settings
}

int main() {
    string filename = "numbers.csv"; // Change to the name of your file
    float avgX, avgY;
    int avgT;

    if (computeAverages(filename, avgX, avgY, avgT)) {
        cout << "Average x: " << avgX << endl;
        cout << "Average y: " << avgY << endl;
        cout << "Average t: " << avgT << endl;

        // Example usage of raw mode for custom key handling
        struct termios oldt;
        setRawMode(oldt);

        char key;
        cout << "Press any key to continue..." << endl;
        read(STDIN_FILENO, &key, 1); // Read one character from stdin

        resetTerminalMode(oldt);
        cout << "You pressed: " << key << endl;

        // Integrate the averages into your robot's movement logic here
        // For example:
        // moveRobot(avgX, avgY, avgT);
    } else {
        cerr << "Failed to compute averages." << endl;
    }

    return 0;
}
