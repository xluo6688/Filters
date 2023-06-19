// InclinometerFilter.cpp : This file contains the 'main' function. Program execution begins and ends there.
// Project: This filter is designed for Joy Inclinometer with P/N 100613522 Issue 01 SN 13/1585
// 1. Initial version, Feb. 2023, Xuanwen Luo

#include <iostream>
#include <vector>
#include <fstream>
#include "Gaussian.h"
#include "Filter1D.h"
#include "fileApp.h"
#include <time.h>
#include <chrono>
#include <iomanip>
#include <ctime>
#include "dataType.h"
#include "Filter2D.h"

using namespace std;

int main()
{
    std::cout << "Inclinometer Filter !\n";

    int dataCol = 4;
    vector<double> data;
    //vector<Gaussian> estimates;
    vector<dataType> estimates;
    vector<dataType> estimates2D;

    time_t nowtime = time(NULL);
    struct tm timeinfo;
    time(&nowtime);
    localtime_s(&timeinfo, &nowtime);
    string outFileName;

    outFileName = "TestData" + to_string(timeinfo.tm_year + 1900) + to_string(timeinfo.tm_mon + 1) + to_string(timeinfo.tm_mday) + to_string(timeinfo.tm_hour) + to_string(timeinfo.tm_min) + to_string(timeinfo.tm_sec) + ".csv";

   // Filter1D inclinometerFilter;
    Filter2D inclinometerFilter2D;

    // LogData_NoMotion_ChangeAngle_xy
    // LogData_Motion_Flat_xy
    // LogData_Motion_AngleChange_xy

    string fileName = "InclinomTestData_MotionChange.csv";
    data = fileApp::readCsvData(fileName, dataCol);
   // estimates = inclinometerFilter.filter(data);

    estimates2D = inclinometerFilter2D.filter2D(data);


    cout << "dataSize = " << data.size();


    if (!fileApp::writeCsvData2D(outFileName, data, estimates2D))
    {
        cout << "Writing Data Failed !" << endl;
    }

    return 0;

}
// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
