#include <cstdio>
#include <delaunator.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

std::vector<double> conepos;
cv::Mat pathmap = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);

cv::Scalar color(0,255,255);


void save_pos(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          conepos.push_back(((double)x)/50-5);
          conepos.push_back(((double)y)/50);
          cv::circle(pathmap, cv::Point(x,y), 5, color, 5);
     }
}
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
        if (k == 'a') color = cv::Scalar(0,255,255);
        if (k == 'z') color = cv::Scalar(255,100,50);

    }

    //triangulation happens here
    delaunator::Delaunator d(conepos);
    for (int i =0; i < d.triangles.size(); i+=3)
    {
        std::cout << "yes" << std::endl;
        cv::line(pathmap, cv::Point(d.coords[2*d.triangles[i]]*50+250, d.coords[2*d.triangles[i] + 1]*50), cv::Point(d.coords[2*d.triangles[i+1]]*50+250, d.coords[2*d.triangles[i+1] + 1]*50), cv::Scalar(0,0,0), 2);
        cv::line(pathmap, cv::Point(d.coords[2*d.triangles[i+1]]*50+250, d.coords[2*d.triangles[i+1] + 1]*50), cv::Point(d.coords[2*d.triangles[i+2]]*50+250, d.coords[2*d.triangles[i+2] + 1]*50), cv::Scalar(0,0,0), 2);
        cv::line(pathmap, cv::Point(d.coords[2*d.triangles[i]]*50+250, d.coords[2*d.triangles[i] + 1]*50), cv::Point(d.coords[2*d.triangles[i+2]]*50+250, d.coords[2*d.triangles[i+2] + 1]*50), cv::Scalar(0,0,0), 2);
    }
    cv::imshow("map", pathmap);
    cv::waitKey(0);
}
