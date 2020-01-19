#include <cstdio>
#include <delaunator.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <unordered_set>

std::vector<double> conepos;
std::vector<cv::Point2i> added_points;
std::vector<cone> all_cones;
std::vector<tracknode> all_nodes;
cv::Mat pathmap = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);

cv::Scalar color(0,255,255);
int colori = 0;

struct cone
{
    cv::Point2i location;
    int type;
};

struct tracknode
{
    cv::Point2i location;

    cone cone1;
    cone cone2;

    tracknode * prev_location;

    int pathnr;
};


void save_pos(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          conepos.push_back(((double)x));
          conepos.push_back(((double)y));
          cone a_cone;
          a_cone.location = cv::Point2i(x,y);
          a_cone.type = colori;
          all_cones.push_back(a_cone);
          if (colori == 0) cv::circle(pathmap, cv::Point(x,y), 5, cv::Scalar(0,255,255), 5);
          if (colori == 1) cv::circle(pathmap, cv::Point(x,y), 5, cv::Scalar(255,100,50), 5);
     }
}
/*
float color_cost

float distance_cost

float angle_cost

float width_cost

*/






int main() {
    /* x0, y0, x1, y1, ... */
    pathmap.setTo(cv::Scalar(255, 255, 255));
    
    cv::namedWindow("map");
    for (int i = 1; i < 10; i++) {
        cv::line(pathmap, cv::Point(0, i * 50), cv::Point(500, i * 50), cv::Scalar(0, 0, 0));
        cv::line(pathmap, cv::Point(i * 50, 0), cv::Point(i * 50, 500), cv::Scalar(0, 0, 0));
    }
    cv::setMouseCallback("map", save_pos);
    while (1) {
        int k;
        cv::imshow("map", pathmap);
        k = cv::waitKey(16);
        if (k == 27) break;
        if (k == 'a') colori = 0;
        if (k == 'z') colori = 1;

    }

    //triangulation happens here
    delaunator::Delaunator d(conepos);
    for (int i =0; i < d.triangles.size(); i+=3)
    {
        std::cout << "yes" << std::endl;
        cv::line(pathmap, cv::Point2i(d.coords[2*d.triangles[i]], d.coords[2*d.triangles[i] + 1]), cv::Point(d.coords[2*d.triangles[i+1]], d.coords[2*d.triangles[i+1] + 1]), cv::Scalar(0,0,0), 2);
        cv::line(pathmap, cv::Point2i(d.coords[2*d.triangles[i+1]], d.coords[2*d.triangles[i+1] + 1]), cv::Point(d.coords[2*d.triangles[i+2]], d.coords[2*d.triangles[i+2] + 1]), cv::Scalar(0,0,0), 2);
        cv::line(pathmap, cv::Point2i(d.coords[2*d.triangles[i]], d.coords[2*d.triangles[i] + 1]), cv::Point(d.coords[2*d.triangles[i+2]], d.coords[2*d.triangles[i+2] + 1]), cv::Scalar(0,0,0), 2);
        
        cv::Point2f pt1((d.coords[2*d.triangles[i]] + d.coords[2*d.triangles[i+1]])/2, (d.coords[2*d.triangles[i]+1] + d.coords[2*d.triangles[i+1]+1])/2);
        cv::Point2f pt2((d.coords[2*d.triangles[i+1]] + d.coords[2*d.triangles[i+2]])/2, (d.coords[2*d.triangles[i+1]+1] + d.coords[2*d.triangles[i+2]+1])/2);
        cv::Point2f pt3((d.coords[2*d.triangles[i]] + d.coords[2*d.triangles[i+2]])/2, (d.coords[2*d.triangles[i]+1] + d.coords[2*d.triangles[i+2]+1])/2);

        std::vector<cv::Point2i>::iterator it;

        it = std::find(added_points.begin(), added_points.end(), pt1);
        if(it != added_points.end())
        {
            
        }
        else
        {
            tracknode a_node;
            a_node.location = pt1;
            a_node.cone1
            all_nodes.push_back(a_node);
        }
        

        added_points.push_back(cv::Point2i((d.coords[2*d.triangles[i]] + d.coords[2*d.triangles[i+1]])/2, (d.coords[2*d.triangles[i]+1] + d.coords[2*d.triangles[i+1]+1])/2));
        added_points.push_back(cv::Point2i((d.coords[2*d.triangles[i+1]] + d.coords[2*d.triangles[i+2]])/2, (d.coords[2*d.triangles[i+1]+1] + d.coords[2*d.triangles[i+2]+1])/2));
        added_points.push_back(cv::Point2i((d.coords[2*d.triangles[i]] + d.coords[2*d.triangles[i+1]])/2, (d.coords[2*d.triangles[i]+1] + d.coords[2*d.triangles[i+1]+1])/2));


    
    }

    for (cv::Point2i &pt : added_points)
    {
        cv::circle(pathmap, pt, 3, cv::Scalar(0,255,0), 4);
    }

    cv::imshow("map", pathmap);
    cv::waitKey(0);
    
}


