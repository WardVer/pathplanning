#include <cmath>
#include <cstdio>
#include <delaunator.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <random>
#include <unordered_set>

#define maxcost 10

std::chrono::steady_clock::time_point begin;
std::chrono::steady_clock::time_point end;

cv::Scalar color(0, 255, 255);
int colori = 0;

struct cone {
    cv::Point2i location;
    int type;
};

struct tracknode {
    cv::Point2i location;

    cone cone1;
    cone cone2;

    cone* leftcone;
    cone* rightcone;

    float pathcost = 1000;
    int inpath = 0;

    tracknode* prevnode;

    int pathnr;
};

std::vector<double> conepos;
std::vector<cv::Point2i> added_points;
std::vector<cone> all_cones;
std::vector<tracknode> all_nodes;
cv::Mat pathmap = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);

float calc_angle(cv::Point2i vec1, cv::Point2i vec2) {

    int dot = vec1.x * vec2.x + vec1.y * vec2.y;
    int det = vec1.x * vec2.y - vec1.y * vec2.x;
    float angle = std::atan2(det, dot);
    return angle;
}

void save_pos(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        conepos.push_back(((double)x));
        conepos.push_back(((double)y));
        cone a_cone;
        a_cone.location = cv::Point2i(x, y);
        a_cone.type = colori;
        all_cones.push_back(a_cone);
        if (colori == 0) cv::circle(pathmap, cv::Point(x, y), 4, cv::Scalar(0, 255, 255), 4);
        if (colori == 1) cv::circle(pathmap, cv::Point(x, y), 4, cv::Scalar(255, 100, 50), 4);
    }
}

void color_cost(tracknode* newnode, tracknode* currentnode) {

    float cone1_angle = calc_angle(newnode->location - currentnode->location, newnode->cone1.location - newnode->location);

    int leftcolor;
    int rightcolor;

    if (cone1_angle <= 0) {
        newnode->leftcone = &newnode->cone1;
        newnode->rightcone = &newnode->cone2;
    } else {
        newnode->leftcone = &newnode->cone2;
        newnode->rightcone = &newnode->cone1;
    }

    if (newnode->leftcone->type == newnode->rightcone->type) {
        newnode->pathcost += 5;
    }

    if (newnode->leftcone->type == 0 && newnode->rightcone->type == 1) {
        newnode->pathcost += 10;
    }
}

void dist_cost(tracknode* newnode, tracknode* currentnode) {
    /*int r = std::rand() % ( 255 );
    int g = std::rand() % ( 255 );
    int b = std::rand() % ( 255 );*/
    newnode->pathcost += cv::norm(newnode->location - currentnode->location) / 25;
    /*std::cout << newnode->pathcost << std::endl;
    cv::circle(pathmap, newnode->location, 4, cv::Scalar(b, g, r), 4);
    cv::imshow("map", pathmap);
    cv::waitKey(0);*/
}

void angle_cost(tracknode* newnode, tracknode* currentnode) {

    float angle;
    if(currentnode->prevnode != NULL)
    {
        angle = calc_angle(newnode->location - currentnode->location, currentnode->location - currentnode->prevnode->location);
    }
    else
    {
        angle = calc_angle(newnode->location - currentnode->location, cv::Point2i(0,-1));
    }
    

    
    newnode->pathcost += 4 * abs(angle);
    
}

/*
void angle_cost

void width_cost

*/

void calc_cost(tracknode* currentnode, std::vector<tracknode*> opennodes) {
    /*cv::circle(pathmap, currentnode->location, 7, cv::Scalar(0, 0, 255), 7);
    cv::imshow("map", pathmap);
    cv::waitKey(0);*/
    for (int i = 0; i < opennodes.size(); i++) {

        if (opennodes.at(i)->inpath != 1) {
            opennodes.at(i)->pathcost = 0;
            color_cost(opennodes.at(i), currentnode);
            dist_cost(opennodes.at(i), currentnode);
            angle_cost(opennodes.at(i), currentnode);
        }
        
        //width_cost();
    }
}

int main() {
    /* x0, y0, x1, y1, ... */
    std::cout << calc_angle(cv::Point2i(1, 2), cv::Point2i(0, 2)) << std::endl;
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
    begin = std::chrono::steady_clock::now();
    delaunator::Delaunator d(conepos);

    for (int i = 0; i < d.triangles.size(); i += 3) {

        cv::line(pathmap, cv::Point2i(d.coords[2 * d.triangles[i]], d.coords[2 * d.triangles[i] + 1]), cv::Point(d.coords[2 * d.triangles[i + 1]], d.coords[2 * d.triangles[i + 1] + 1]), cv::Scalar(0, 0, 0), 1);
        cv::line(pathmap, cv::Point2i(d.coords[2 * d.triangles[i + 1]], d.coords[2 * d.triangles[i + 1] + 1]), cv::Point(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1]), cv::Scalar(0, 0, 0), 1);
        cv::line(pathmap, cv::Point2i(d.coords[2 * d.triangles[i]], d.coords[2 * d.triangles[i] + 1]), cv::Point(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1]), cv::Scalar(0, 0, 0), 1);

        cv::Point2i pt1((d.coords[2 * d.triangles[i]] + d.coords[2 * d.triangles[i + 1]]) / 2, (d.coords[2 * d.triangles[i] + 1] + d.coords[2 * d.triangles[i + 1] + 1]) / 2);
        cv::Point2i pt2((d.coords[2 * d.triangles[i + 1]] + d.coords[2 * d.triangles[i + 2]]) / 2, (d.coords[2 * d.triangles[i + 1] + 1] + d.coords[2 * d.triangles[i + 2] + 1]) / 2);
        cv::Point2i pt3((d.coords[2 * d.triangles[i]] + d.coords[2 * d.triangles[i + 2]]) / 2, (d.coords[2 * d.triangles[i] + 1] + d.coords[2 * d.triangles[i + 2] + 1]) / 2);

        std::vector<cv::Point2i>::iterator it;

        it = std::find(added_points.begin(), added_points.end(), pt1);
        if (it == added_points.end()) {
            tracknode a_node;
            for (cone& test_cone : all_cones) {
                if (test_cone.location == cv::Point2i(d.coords[2 * d.triangles[i]], d.coords[2 * d.triangles[i] + 1])) {
                    a_node.cone1 = test_cone;
                }
                if (test_cone.location == cv::Point2i(d.coords[2 * d.triangles[i + 1]], d.coords[2 * d.triangles[i + 1] + 1])) {
                    a_node.cone2 = test_cone;
                }
            }
            a_node.location = pt1;
            all_nodes.push_back(a_node);
            added_points.push_back(pt1);
        }

        it = std::find(added_points.begin(), added_points.end(), pt2);
        if (it == added_points.end()) {
            tracknode a_node;
            for (cone& test_cone : all_cones) {
                if (test_cone.location == cv::Point2i(d.coords[2 * d.triangles[i + 1]], d.coords[2 * d.triangles[i + 1] + 1])) {
                    a_node.cone1 = test_cone;
                }
                if (test_cone.location == cv::Point2i(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1])) {
                    a_node.cone2 = test_cone;
                }
            }
            a_node.location = pt2;
            all_nodes.push_back(a_node);
            added_points.push_back(pt2);
        }

        it = std::find(added_points.begin(), added_points.end(), pt3);
        if (it == added_points.end()) {
            tracknode a_node;
            for (cone& test_cone : all_cones) {
                if (test_cone.location == cv::Point2i(d.coords[2 * d.triangles[i]], d.coords[2 * d.triangles[i] + 1])) {
                    a_node.cone1 = test_cone;
                }
                if (test_cone.location == cv::Point2i(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1])) {
                    a_node.cone2 = test_cone;
                }
            }
            a_node.location = pt3;
            all_nodes.push_back(a_node);
            added_points.push_back(pt3);
        }
    }
    /*
    for (tracknode& nde : all_nodes) {

        cv::Scalar color1(0, 0, 0);
        cv::Scalar color2(0, 0, 0);
        cv::circle(pathmap, nde.location, 4, cv::Scalar(255, 0, 255), 4);

        if (nde.cone1.type == 0)
        {
                color1 = cv::Scalar(0,255,255); 
        }
        else
        {
                color1 = cv::Scalar(255,100,50); 
        }
        
        if (nde.cone2.type == 0)
        {
                color2 = cv::Scalar(0,255,255);         
        }
        else
        {
                color2 = cv::Scalar(255,100,50); 
        }

        cv::circle(pathmap, nde.cone1.location, 4, color1, 4);
        cv::circle(pathmap, nde.cone2.location, 4, color2, 4);
        cv::imshow("map", pathmap);
        cv::waitKey(0);
    }
    cv::imshow("map", pathmap);
    cv::waitKey(0);*/

    tracknode carnode;
    tracknode* currentnode;
    carnode.pathnr = 0;
    carnode.location = cv::Point2i(250, 500);
    carnode.prevnode = NULL;
    carnode.inpath = 1;
    std::vector<tracknode*> opennodes;
    currentnode = &carnode;

    std::vector<tracknode*> finalpath;
    std::vector<cone*> finalleft;
    std::vector<cone*> finalright;

    for (int o = 0; o < 500; o++) {

        finalpath.push_back(currentnode);

        opennodes.clear();
        for (int i = 0; i < all_nodes.size(); i++) 
        {
            if (all_nodes[i].inpath == 0)
                opennodes.push_back(&all_nodes[i]);
        }
        std::vector<tracknode*>::iterator it;
        it = std::find(opennodes.begin(), opennodes.end(), currentnode);
        if (it != opennodes.end())
            opennodes.erase(it);

        

        calc_cost(currentnode, opennodes);

        tracknode* bestnode = opennodes.at(0);

        for (tracknode* testnode : opennodes) {
            if (testnode->inpath == 1) continue;
            if (testnode->pathcost < bestnode->pathcost) {
                bestnode = testnode;
            }
        }
        if (bestnode->pathcost >= 10) break;
        bestnode->inpath = 1;
        std::cout << currentnode->location << bestnode->location << std::endl;

        bestnode->prevnode = currentnode;

        currentnode = bestnode;
    }
    std::cout << "here" << std::endl;
    for (tracknode* final_node : finalpath) {
        if (final_node->prevnode != NULL) {
            std::cout << final_node->location << final_node->prevnode->location << std::endl;

            cv::line(pathmap, final_node->location, final_node->prevnode->location, cv::Scalar(0, 255, 0), 2);
            if( final_node->prevnode->leftcone != NULL)
            {
                cv::line(pathmap, final_node->leftcone->location, final_node->prevnode->leftcone->location, cv::Scalar(255, 100, 50), 2);
                cv::line(pathmap, final_node->rightcone->location, final_node->prevnode->rightcone->location, cv::Scalar(0, 255, 255), 2);
            }
            /*cv::imshow("map", pathmap);
            cv::waitKey(0);*/
        }
    }

    end = std::chrono::steady_clock::now();
    std::cout << "show time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    while (1) {
        cv::imshow("map", pathmap);
        cv::waitKey(0);
    }
}
