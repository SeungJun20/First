/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   newcppsimpletest.cpp
 * Author: jun
 *
 * Created on February 11, 2020, 11:26 AM
 */

#include <stdlib.h>
#include <iostream>
#include <iostream>
void test1() {
    std::cout << "newcppsimpletest test 1" << std::endl;
}

void test2() {
    std::cout << "newcppsimpletest test 2" << std::endl;
    std::cout << "%TEST_FAILED% time=0 testname=test2 (newcppsimpletest) message=error message sample" << std::endl;
}

int main(int argc, char** argv) {
     double dt;
    double time=0;
    
    sensors::SensorPtr Sensor;
    sensors::ImuSensorPtr IMU;
    double IMU_Update;
    
    double m1 = 24.0;
    double m2 = 4.0;
    double m3 = 2.0;
    double m4 = 12.3;
    double Mb = (m1+m2+m3+m4);
    
    double L1 = -0.085;
    double L2 = 0.034;
    double L3 = 0.5;
    double L4 = 0.65;
    double Lcg = (m1*L1+m2*L2+m3*L3+m4*L4)/Mb;
    
    
    double d = 0.55;
    
    double I1 = 0.2105;
    double I2 = 0.023;
    double I3 = 0.167;
    double I4 = 0.033;
    double Icg = I1+I2+I3+I4+m1*pow(Lcg-L1,2)+m2*pow(Lcg-L2,2)+m3*pow(Lcg-L3,2)+m4*pow(Lcg-L4,2);
    
    
    double Mw = 3.7;
    double Rw = 0.20;
    double Jw = 0.116;
    double g =9.8;
    double np = 13.715;
    double Jm = 1.412*pow(10,-4);
    double Bm = 3.78*pow(10,-4);
    double Kt = 3.756*pow(10,-2);
    double Ke = 3.756*pow(10,-2);
    double Rm = 0.0788;
    double F_d = 0;
    double Tw = 0;
    
    
    
     double e22 = Icg + Mb*pow(Lcg,2)+2*Jm*pow(np,2);
    double e24 = Mb*Rw*Lcg*-2*Jm*pow(np,2);
    double e42 = Mb*Lcg*-2*Jm*pow(np,2);
    double e44 = 2*Jw+(2*Mw+Mb)*pow(Rw,2)+2*Jm*pow(np,2);
    
    
    
    
      double E[4][4] = {
        {1, 0, 0, 0},
        {0, e22, 0, e24},
        {0, 0, 1, 0},
        {0, e42, 0, e44}
    };
    
   
    double H [4][4];
    
    H = Inverse(E,inverse,4);
    
    
    
    
    
    std::cout << H << std::endl;
    std::cout << "%SUITE_STARTED%" << std::endl;

    std::cout << "%TEST_STARTED% test1 (newcppsimpletest)" << std::endl;
    test1();
    std::cout << "%TEST_FINISHED% time=0 test1 (newcppsimpletest)" << std::endl;

    std::cout << "%TEST_STARTED% test2 (newcppsimpletest)\n" << std::endl;
    test2();
    std::cout << "%TEST_FINISHED% time=0 test2 (newcppsimpletest)" << std::endl;

    std::cout << "%SUITE_FINISHED% time=0" << std::endl;

    return (EXIT_SUCCESS);
}

