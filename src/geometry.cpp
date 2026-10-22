/*
░GEOMETRY░
░GEOMETRY░
░GEOMETRY░

geometry.cpp
*/

#include "geometry.h"
#include <limits>
#include <cmath>

double clamp(double n) {
    if (n < 0.0) return 0.0;
    if (n > 1.0) return 1.0;
    return n;
}

double trapz(const Eigen::VectorXd&        dt,
             const std::vector<double>&    values) {
    
    double area = 0.0;
    for (int i = 1; i < (int)values.size(); ++i) {
        area += (dt[i] - dt[i-1]) * (values[i] + values[i-1]) / 2.0;
    }
    return area;
}

DistanceResult distanceToSegment(
    const Eigen::Vector3d& P1, const Eigen::Vector3d& Q1,
    const Eigen::Vector3d& P2, const Eigen::Vector3d& Q2) {

    Eigen::Vector3d D1 = Q1 - P1;
    Eigen::Vector3d D2 = Q2 - P2;
    Eigen::Vector3d R = P1 - P2;

    double a = D1.dot(D1);
    double b = D1.dot(D2);
    double c = D1.dot(R);
    double e = D2.dot(D2);
    double f = D2.dot(R);

    double d = a*e - b*b;
    if (std::abs(d) < 1e-9) d = 1e-9;

    double s = clamp((b*f - c*e) / d);

    double t = (b*s + f) / e;

    if (t < 0.0) {
        t = 0.0;
        s = clamp(-c / a);
    } else if (t  > 1.0) {
        t = 1.0;
        s = clamp((b - c) / a);
    }

    Eigen::Vector3d C_h = P1 + D1*s;
    Eigen::Vector3d C_r = P2 + D2*t;

    //std::cout << "a=" << a << "\nb=" << b << "\nc=" << c << "\nd=" << d << "\ne=" << e << "\nf=" << f << "\n";

    double distance = (C_h - C_r).norm();

    return {distance, C_h, C_r};
}

std::pair<std::optional<DistanceResult>, int> distanceToSkeleton(
    const Skeleton&        skeleton,
    const Eigen::Vector3d& P2,
    const Eigen::Vector3d& Q2) {

    int min_ind = 0;
    double min_dist = std::numeric_limits<double>::max();
    std::optional<DistanceResult> min_result;
    
    for (int i = 0; i < (int)SKEL_INDEX.size(); i++) {
        int ind1 = SKEL_INDEX[i][0];
        int ind2 = SKEL_INDEX[i][1];

        // guard against skeleton shorter than 21 joints

        if (ind1 < 0 || ind2 < 0 ||
            ind1 >= (int)skeleton.size() || ind2 >= (int)skeleton.size()) {
            continue;
        }

        Eigen::Vector3d P1(skeleton[ind1][0], skeleton[ind1][1], skeleton[ind1][2]);
        Eigen::Vector3d Q1(skeleton[ind2][0], skeleton[ind2][1], skeleton[ind2][2]);
        
        if (P1.hasNaN() || Q1.hasNaN()) continue; // skip iteration if NaN
        //std::cout << "Entering dts, " << min_dist << "\n";
        //std::cout << "P1=" << P1 << "\n Q1=" << Q1 << "\n P2=" << P2 << "\n Q2=" << Q2 << "\n";

        DistanceResult result = distanceToSegment(P1, Q1, P2, Q2);
        //std::cout << "result.distance=" << result.distance << "\n\n";

        if (result.distance < min_dist) {
            min_dist = result.distance;
            min_ind = i;
            min_result = result;
        }
    }

    return {min_result, min_ind};
}