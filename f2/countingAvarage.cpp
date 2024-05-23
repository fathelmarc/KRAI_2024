#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <deque>

#include <unistd.h>
#include <termios.h>

using namespace std;

const size_t MAX_ENTRIES = 132; // Maximum number of entries to consider

// Function to process each line and add it to the buffer
void processLine(const string& line, deque<vector<float>>& buffer) {
    stringstream lineStream(line);
    string cell;
    vector<float> values;

    // Split the line by commas and convert to float
    while (getline(lineStream, cell, ',')) {
        values.push_back(stof(cell));
    }

    // Add the new values to the buffer
    if (values.size() >= 3) {
        if (buffer.size() >= MAX_ENTRIES) {
            buffer.pop_front(); // Remove the oldest entry
        }
        buffer.push_back(values); // Add the newest entry
    }
}

// Function to get a key press
string getKeyPress() {
    char key;
    struct termios oldt, newt;

    // Save current terminal settings
    tcgetattr(STDIN_FILENO, &oldt);

    // Set terminal to non-canonical mode (no buffering)
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read a single character
    read(STDIN_FILENO, &key, 1);

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return string(1, key);
}

// Function to compute and print the averages
void computeAndPrintAverages(const deque<vector<float>>& buffer) {
    if (buffer.empty()) {
        cout << "No data to process." << endl;
        return;
    }

    float sumX = 0.0;
    float sumY = 0.0;
    int sumT = 0;
    int count = buffer.size();

    for (const auto& values : buffer) {
        sumX += values[0];
        sumY += values[1];
        sumT += static_cast<int>(values[2]);
    }

    float avgX = sumX / count;
    float avgY = sumY / count;
    int avgT = sumT / count; // Integer division

    cout << "Average x: " << avgX << endl;
    cout << "Average y: " << avgY << endl;
    cout << "Average t: " << avgT << endl;
}

// Function to read the file and update the buffer
void updateBuffer(const string& filename, deque<vector<float>>& buffer) {
    ifstream inputFile(filename);

    if (!inputFile) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    string line;
    while (getline(inputFile, line)) {
        processLine(line, buffer);
    }

    inputFile.close();
}

int main() {
    string filename = "numbers.txt"; // Change to the name of your file
    deque<vector<float>> buffer;

    // Initial load of the file into the buffer
    updateBuffer(filename, buffer);

    cout << "Press '1' to compute averages or 'q' to quit..." << endl;

    while (true) {
        string key = getKeyPress();

        if (key == "1") {
            computeAndPrintAverages(buffer);
        } else if (key == "q") {
            break;
        }
    }

    return 0;
}
