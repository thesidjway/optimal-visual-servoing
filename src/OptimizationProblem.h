#include <cstdlib>
#include <vector>
#include <ctime>
#include <iostream>
#include <ceres/ceres.h>

struct RangeDataTuple {
	RangeDataTuple (double median_dist, double bearing, double width) : median_dist(median_dist), bearing(bearing), width(width) {}
	double median_dist;
	double bearing;
	double width;
};

class OptimizationProblem {
private:

  struct DistanceError {
  DistanceError(RangeDataTuple data_tuple)
      : data_tuple(data_tuple) {}

  template <typename T>
  bool operator()(const T* const r1,
                  const T* const theta1,
                  T* residuals) const {
    residuals[0] =  T(data_tuple.median_dist) - r1[0];
    return true;
    }
    static ceres::CostFunction* Create(RangeDataTuple data_tuple) {
    	return (new ceres::AutoDiffCostFunction<DistanceError, 1, 1, 1>(new DistanceError(data_tuple)));
   }

  RangeDataTuple data_tuple;
};
	ceres::Problem problem;
	double r1 = 0.5;
    double theta1 = 10; 
public:
	OptimizationProblem() {}
	~OptimizationProblem() {}
	void addRangeFactor(RangeDataTuple& tuple);
	void generateData(std::vector<RangeDataTuple>& gen_data);
	void optimizeGraph();
};