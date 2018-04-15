#include <utility>
#include <vector>
#include <queue>
#include <functional>
#include <math.h> 
#include <iostream>

#include "umap_util.hpp"
#include "wp_planning.hpp"

using namespace std;
using namespace umap_utility;

namespace wp
{
    double distance(double x1, double y1, double x2, double y2)
    {
        return sqrt( (pow(x1-x2,2)+ pow(y1-y2,2)) );
    }
    
    bool has_collision(iPair p, vector<umap_utility::ellipse_desc> &obstacles, int obstacle_count)
    {
        int i, j;
        double x, y, h, k, rx, ry;
        x = p.first;
        y = p.second;
        for (i = 0; i < obstacle_count; i++)
        {
            h = obstacles[i].GPe.ptr<double>(0)[0];
            k = obstacles[i].GPe.ptr<double>(1)[0];
            rx = obstacles[i].BSe.ptr<double>(0)[0];
            ry = obstacles[i].BSe.ptr<double>(1)[0];
            // cout << h << " " << k << " " << rx << " " << ry << endl;
            // cout << x << " " << y << endl;
            if (ry < 0.5) ry = 0.5;
            double calc = (x - h) * (x - h) / (rx * rx) + (y - k) * (y - k) / (ry * ry);
            // cout << calc << endl;
            if (calc < 1.0)
            {   
                // cout << h << " " << k << " " << rx << " " << ry << endl;
                return true;
            }
        }
        return false;
    }

    vector<iPair> waypoint_checking(vector<iPair> waypoints, vector<ellipse_desc> &obstacles, int obstacle_count)
    {
        int i, j, k, selected_index;
        double min_dist;
        double cur_dist = 0;
        double avoiding_dist = 10;
        queue<iPair> wps_to_check;
        vector<iPair> waypoints_out;
        vector< vector<iPair> > allowed_wps(waypoints.size());
        iPair current_check_point, current_point;
        current_point = waypoints[0];
        for (i = 0; i < waypoints.size(); i++)
        {
            wps_to_check.push(waypoints[i]);
            while (!wps_to_check.empty())
            {
                current_check_point = wps_to_check.front();
                // cout << "current waypoint " << current_check_point.first << " " << current_check_point.second << endl;
                wps_to_check.pop();
                if (has_collision(current_check_point, obstacles, obstacle_count))
                {
                    k = 1;
                    while(has_collision(make_pair(current_check_point.first - avoiding_dist*k, current_check_point.second), obstacles, obstacle_count))
                    {
                        k++;
                    }
                    wps_to_check.push(make_pair(current_check_point.first - avoiding_dist*k, current_check_point.second)); // path to left
                    
                    k = 1;
                    while(has_collision(make_pair(current_check_point.first + avoiding_dist*k, current_check_point.second), obstacles, obstacle_count))
                    {
                        k++;
                    }
                    wps_to_check.push(make_pair(current_check_point.first + avoiding_dist*k, current_check_point.second)); // path to right 
                }
                else
                {
                    allowed_wps[i].push_back(current_check_point);
                }
            }
            selected_index = 0;
            min_dist = cur_dist + distance(allowed_wps[i][0].first, allowed_wps[i][0].second, current_point.first, current_point.second);
            // cout << "Allow size " << allowed_wps[i].size() << endl;
            for( j = 0 ; j < allowed_wps[i].size() ; j++)
            {
                // cout << "Allow waypoint "<< allowed_wps[i][j].first << " " << allowed_wps[i][j].second << endl;
                if (min_dist > (cur_dist + distance(allowed_wps[i][j].first, allowed_wps[i][j].second, current_point.first, current_point.second)))
                {
                    min_dist = cur_dist + distance(allowed_wps[i][j].first, allowed_wps[i][j].second, current_point.first, current_point.second);
                    selected_index = j;
                }
            }
            cur_dist = min_dist;
            current_point = allowed_wps[i][selected_index];
            waypoints_out.push_back(current_point);
        }
    return waypoints_out;
    }
}