/*
░█▀▀░█▀▀░█▀█░█▄█░█▀▀░▀█▀░█▀▄░█░█
░█░█░█▀▀░█░█░█░█░█▀▀░░█░░█▀▄░░█░
░▀▀▀░▀▀▀░▀▀▀░▀░▀░▀▀▀░░▀░░▀░▀░░▀░

geometry.h - Numerical integration helper used by the stop-checking code.
*/

#pragma once
#include <vector>
#include <Eigen/Core>

// Trapezoidal integration
double trapz(const Eigen::VectorXd&        dt,
             const std::vector<double>&    values);
