#ifndef IN_POLYGON_CHECK_H
#define IN_POLYGON_CHECK_H

#include <bits/stdc++.h>
#include <iostream>

using namespace std;

class Check {		
	public:
		// // Declare and define Point struct
		struct Point {
			double x, y;
		};

		// // Declare and define Point struct
		struct line {
			Point p1, p2;
		};

		bool onLine(line l1, Point p);

		int direction(Point a, Point b, Point c);

		bool isIntersect(line l1, line l2);

		bool checkInside(Point poly[], int n, Point p);
};
#endif