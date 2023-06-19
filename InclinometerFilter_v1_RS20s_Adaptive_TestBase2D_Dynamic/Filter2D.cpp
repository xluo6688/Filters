#include "Filter2D.h"
#include <vector>
#include "dataType.h"
#include <Eigen/Dense>
#include <iostream>

void Filter2D::predict(
    const Eigen::VectorXd& x,
    Eigen::VectorXd& x_hat,
    const Eigen::MatrixXd& P,
    Eigen::MatrixXd& P_hat,
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& Q)
{
    x_hat = F * x;
    P_hat = F * P * F.transpose() + Q;
}


void Filter2D::update(
    Eigen::VectorXd& x,
    const Eigen::VectorXd& x_hat,
    Eigen::MatrixXd& P,
    const Eigen::MatrixXd& P_hat,
    const Eigen::VectorXd& z,
    const Eigen::RowVectorXd& H,
    const Eigen::MatrixXd& R,
    Eigen::VectorXd& c_var)
{
    Eigen::VectorXd S;
    Eigen::MatrixXd Si;
    Eigen::MatrixXd K;
    Eigen::VectorXd y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x.rows(), x.rows());

    S = H * P_hat * H.transpose() + R;
    Si = S.inverse();
    K = P_hat * H.transpose() * Si;
    y = z - H * x_hat;
    c_var = y.transpose() * Si * y;
    x = x_hat + K * y;
    P = (I - K * H) * P_hat;
}

vector<dataType> Filter2D::filter2D(vector<double> data) {

    vector<dataType> filteredData;
    dataType outputData;
    double dt = 0.05;//0.03, 0.05, 0.5, 1.0
    int n = 2; // Number of states
    int m = 1; // Number of measurements
    //double process_std = 0.05;
    double process_std = 0.005;
    double measurement_std = 2; // 2

    Eigen::VectorXd x(n);
    x << data[0], 
         0;
    Eigen::VectorXd x_hat(n, 1);
    Eigen::MatrixXd F(n, n);
    F << 1.0, dt, 
         0.0, 1.0;
    Eigen::MatrixXd P(n, n);
    P << 0.1, 0.0,
         0.0, 0.1;
    Eigen::MatrixXd P_hat(n, n);
    Eigen::RowVectorXd H(n);
    H << 1.0, 0.0;
    Eigen::MatrixXd Q(n, n);
    Eigen::MatrixXd R(1,1);
    R << pow(measurement_std, 2);

    Eigen::VectorXd z(1);exc
        exc

    double normRersidual;
    Eigen::VectorXd c_var;
    double process_var;    

    for (int i = 0; i <(int) data.size(); i++)
    {
        process_var = pow(process_std, 2);
       
        Q << 0.25*pow(dt,4) * process_var, 0.5*pow(dt,3) * process_var,
             0.5 * pow(dt, 3) * process_var, pow(dt,2)* process_var;

        z << data[i];
        predict(x, x_hat, P, P_hat, F, Q); 
        update(x, x_hat, P, P_hat, z, H, R, c_var);
        normRersidual = c_var(0);

        //if (normRersidual > 0.1) //RBoom_Angle - A. LogData_Franklin_Pass1 
        //    process_std = 50;
        //else if (normRersidual > 0.05)
        //    process_std = 5;
        //else if (normRersidual > 0.02)
        //    process_std = 0.5;
        //else if (normRersidual > 0.015)
        //    process_std = 0.2;
        //else
        //    process_std = 0.0005;

        //if (normRersidual > 0.1) //RBoom_Angle - B. LogData_Franklin_Pass1
        //    process_std = 10;
        //else if (normRersidual > 0.05)
        //    process_std = 1;
        //else if (normRersidual > 0.02)
        //    process_std = 0.01;
        //else if (normRersidual > 0.015)
        //    process_std = 0.001;
        //else
        //    process_std = 0.0005;


        //if (normRersidual > 5) //PRSTiltSensorEng - A. LogData_Franklin_Pass1
        //    process_std = 10;
        //else if (normRersidual > 2)
        //    process_std = 1;
        //else if (normRersidual > 1)
        //    process_std = 0.05;
        //else if (normRersidual > 0.5)
        //    process_std = 0.005;
        //else if (normRersidual > 0.1)
        //    process_std = 0.001;
        //else if (normRersidual > 0.05)
        //    process_std = 0.0001;
        //else
        //    process_std = 0.00005;


        //if (normRersidual > 5) //PRSTiltSensorEng - B. LogData_Franklin_Pass1
        //    process_std = 10;
        //else if (normRersidual > 2)
        //    process_std = 0.1;
        //else if (normRersidual > 1)
        //    process_std = 0.05;
        //else if (normRersidual > 0.5)
        //    process_std = 0.005;
        //else if (normRersidual > 0.1)
        //    process_std = 0.001;
        //else if (normRersidual > 0.05)
        //    process_std = 0.0001;
        //else
        //    process_std = 0.00005;



        if (normRersidual > 5) //PRSTiltSensorEng - C. LogData_Franklin_Pass1
            process_std = 1;
        else if (normRersidual > 2)
            process_std = 0.1;
        else if (normRersidual > 1)
            process_std = 0.05;
        else if (normRersidual > 0.5)
            process_std = 0.005;
        else if (normRersidual > 0.1)
            process_std = 0.001;
        else if (normRersidual > 0.05)
            process_std = 0.0001;
        else
            process_std = 0.00005;





        //if (normRersidual > 500) // LogData_Motion_AngleChange_xy 
        //    process_std = 100000.2;
        //else if (normRersidual > 200)
        //    process_std = 10000.2;
        //else if (normRersidual > 100)
        //    process_std = 5000.2;
        //else if (normRersidual > 0.5)
        //    process_std = 50.2;
        //else if (normRersidual > 0.04) // 0.03, 0.04, 0.05
        //    process_std = 0.002;  // increase process_std when scan frequency increases. 500.2
        //else
        //    process_std = 0.0005;       




        outputData.sensorData2D.filterData = x(0, 0);
        outputData.sensorData2D.normRersidual = normRersidual;
        filteredData.push_back(outputData);
    }  

    return filteredData;

}