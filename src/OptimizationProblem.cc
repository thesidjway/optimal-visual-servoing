#include "OptimizationProblem.h"

void generateData(std::vector<RangeDataTuple>& gen_data) {
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

void addRangeFactor(RangeDataTuple& tuple) {
	std::cout << tuple.median_dist << " " << tuple.bearing << " " << tuple.width << std::endl;
}

int main(int argc, char** argv) {
	std::vector<RangeDataTuple> gen_data;
	generateData(gen_data);
	for (unsigned int i = 0 ; i < gen_data.size() ; i++ ) {
		addRangeFactor(gen_data[i]);
	}
}
