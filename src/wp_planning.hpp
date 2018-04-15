#include <utility>
#include <vector>
#include <list>

#include "umap_util.hpp"

using namespace std;

namespace wp
{
    typedef pair<double, double> iPair;
    double distance(double x1, double y1, double x2, double y2);
    bool has_collision(pair<double, double> p, vector<umap_utility::ellipse_desc> &obstacles, int obstacle_count);
    vector< pair<double,double> > waypoint_checking(vector< pair<double, double> > waypoints, vector<umap_utility::ellipse_desc>& obstacles, int obstacle_count);
}