#ifndef IN_POLYGON_CHECK_H
#define IN_POLYGON_CHECK_H

#include <bits/stdc++.h>
#include <ros/ros.h>

struct Point {
	int x, y;
};

struct line {
	Point p1, p2;
};

bool onLine(line, Point);

int direction(Point, Point, Point);

bool isIntersect(line, line);

bool checkInside(Point, int , Point)

#endif