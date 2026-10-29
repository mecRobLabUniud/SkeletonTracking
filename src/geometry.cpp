/*
░█▀▀░█▀▀░█▀█░█▄█░█▀▀░▀█▀░█▀▄░█░█
░█░█░█▀▀░█░█░█░█░█▀▀░░█░░█▀▄░░█░
░▀▀▀░▀▀▀░▀▀▀░▀░▀░▀▀▀░░▀░░▀░▀░░▀░

geometry.cpp - Numerical integration helper used by the stop-checking code.
*/

#include "geometry.h"

double trapz(const Eigen::VectorXd&        dt,
             const std::vector<double>&    values) {
    
    double area = 0.0;
    for (int i = 1; i < (int)values.size(); ++i) {
        area += (dt[i] - dt[i-1]) * (values[i] + values[i-1]) / 2.0;
    }
    return area;
}
