#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>

struct Point {
    double x;
    double y;
};

// 计算两点之间的距离
double distance(const Point& p1, const Point& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

// 计算点到直线的距离
double perpendicularDistance(const Point& p, const Point& start, const Point& end) {
    double area = std::abs((end.x - start.x) * (p.y - start.y) - (p.x - start.x) * (end.y - start.y));
    double base = distance(start, end);
    return area / base; 
}

// 使用 RDP 算法简化曲线
void rdp(const std::vector<Point>& pointList, double epsilon, std::vector<Point>& simplified) {
    double dmax = 0.0;
    int index = 0;
    int end = pointList.size() - 1;

    for (int i = 1; i < end; ++i) {
        double d = perpendicularDistance(pointList[i], pointList[0], pointList[end]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }

    if (dmax > epsilon) {
        std::vector<Point> recResults1;
        std::vector<Point> recResults2;

        std::vector<Point> firstLine(pointList.begin(), pointList.begin() + index + 1);
        std::vector<Point> lastLine(pointList.begin() + index, pointList.end());

        rdp(firstLine, epsilon, recResults1);
        rdp(lastLine, epsilon, recResults2);

        simplified.assign(recResults1.begin(), recResults1.end() - 1);
        simplified.insert(simplified.end(), recResults2.begin(), recResults2.end());
    } else {
        simplified.clear();
        simplified.push_back(pointList[0]);
        simplified.push_back(pointList[end]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rdp");
    std::vector<Point> points = {{0, 0}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 8}, {8, 9}, {9, 10}};
    std::vector<Point> simplified;
    double epsilon = 1.0; // 设置阈值

    rdp(points, epsilon, simplified);

    ROS_INFO("Original points:");
    for (const auto& point : points) {
        ROS_INFO_STREAM("(" << point.x << ", " << point.y << ")");
    }

    std::cout << "\nSimplified points:" << std::endl;
    for (const auto& point : simplified) {
        ROS_INFO_STREAM("(" << point.x << ", " << point.y << ")");
    }

    return 0;
}
