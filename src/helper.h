#ifndef HELPER_H
#define HELPER_H

#include <math.h>
#include <vector>
#include <string>
#include "spline.h"

using namespace std;

double deg2rad(double x);
double rad2deg(double x);
std::string hasData(std::string s);
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
std::vector<double> getXY_splines(double s, double d, const tk::spline &s_xs, const tk::spline &s_ys);

#endif