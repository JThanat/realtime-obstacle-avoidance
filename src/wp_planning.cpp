#include <utility>
#include <vector>
#include <queue>
#include <functional>
#include <math.h> 

#include "umap_util.hpp"
#include "wp_planning.hpp"

using namespace std;

namespace wp
{
    double distance(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
    }
    
    bool has_collision(iPair p, vector<umap_utility::ellipse_desc> &obstacles)
    {
        int i, j, k;
        double x, y, h, k, rx, ry;
        x = p.first;
        y = p.second;

        for (i = 0; i < obstacles.size(); i++)
        {
            h = obstacles[i].GPe.ptr<double>(0)[0];
            k = obstacles[i].GPe.ptr<double>(1)[0];
            rx = obstacles[i].BSe.ptr<double>(0)[0];
            ry = obstacles[i].BSe.ptr<double>(1)[0];
            if ((x - h) * (x - h) / (rx * rx) + (y - k) * (y - k) / (ry * ry) < 1)
            {
                return true;
            }
        }
        return false;
    }

    vector<iPair> waypoint_checking(vector<iPair> waypoints, vector<umap_utility::ellipse_desc> &obstacles)
    {
        int i, j, k, selected_index;
        double min_weight;
        double current_weight = 0;
        queue<iPair> wps_to_check;
        vector<iPair> waypoints_out;
        vector<vector<iPair>> allowed_wps(waypoints.size());
        iPair current_check_point, current_point;
        current_point = waypoints[0];

        for (i = 0; i < waypoints.size(); i++)
        {
            wps_to_check.push(waypoints[i]);
            while (!wps_to_check.empty())
            {
                current_check_point = wps_to_check.front();
                wps_to_check.pop();
                if (has_collision(current_check_point, obstacles))
                {
                    wps_to_check.push(make_pair(current_check_point.first - 0.5, current_check_point.second)); // path to left
                    wps_to_check.push(make_pair(current_check_point.first + 0.5, current_check_point.second)); // path to right 
                }
                else
                {
                    allowed_wps[i].push_back(current_check_point);
                }
            }
            selected_index = 0;
            min_weight = current_weight + distance(allowed_wps[i][0].first, current_point.first, allowed_wps[i][0].second, current_point.second);
            for( j = 0 ; j < allowed_wps[i].size() ; j++)
            {
                if (min_weight > (current_weight + distance(allowed_wps[i][j].first, current_point.first, allowed_wps[i][j].second, current_point.second)))
                {
                    min_weight = current_weight + distance(allowed_wps[i][j].first, current_point.first, allowed_wps[i][j].second, current_point.second);
                    selected_index = j;
                }
            }
            current_weight = min_weight;
            current_point = allowed_wps[i][selected_index];
            waypoints_out.push_back(current_point);
        }
    return waypoints_out;
    }
}