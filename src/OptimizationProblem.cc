#include "OptimizationProblem.h"

void OptimizationProblem::generateData(std::vector<RangeDataTuple>& gen_data) {
	srand(time(NULL));
	int angle_init = 0;
	int angle_final = 0;
	while (angle_final < 360) {
		double dist = rand() % 10 + 1;
		int width = rand() % 30;
		angle_final = std::min(angle_init + width, 360);
		gen_data.push_back( RangeDataTuple(dist, (angle_final + angle_init) / 2.0, angle_final - angle_init ));
		angle_init = angle_final;
	}
}



void OptimizationProblem::addRangeFactor(RangeDataTuple& tuple) {
	std::cout << tuple.median_dist << " " << tuple.bearing << " " << tuple.width << std::endl;
	ceres::CostFunction* cost_function =
      DistanceError::Create(tuple);
    problem.AddResidualBlock(cost_function, 
    					   NULL /* squared loss */,
                           &r1,
                           &theta1);

}

void OptimizationProblem::optimizeGraph() {
	ceres::Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << r1 << " and " << theta1 << std::endl;
}

int main(int argc, char** argv) {
	OptimizationProblem opt_problem;
	std::vector<RangeDataTuple> gen_data;
	opt_problem.generateData(gen_data);
	for (unsigned int i = 0 ; i < gen_data.size() ; i++ ) {
		opt_problem.addRangeFactor(gen_data[i]);
	}
	opt_problem.optimizeGraph();
}
