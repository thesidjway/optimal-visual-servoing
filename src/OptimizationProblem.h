#include <cstdlib>
#include <vector>
#include <ctime>
#include <iostream>

struct RangeDataTuple {
	RangeDataTuple (double median_dist, double bearing, double width) : median_dist(median_dist), bearing(bearing), width(width) {}
	double median_dist;
	double bearing;
	double width;
};