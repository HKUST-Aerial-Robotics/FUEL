#include <iostream>
#include <fstream>
#include <ros/ros.h>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "write_tsp");
  ros::NodeHandle node("~");

  ofstream tsp_file("/home/boboyu/workspaces/plan_ws/src/fast_planner/exploration_manager/resource/"
                    "test.tsp");

  // write the problem into file
  // NAME : pr2392
  // COMMENT : 2392-city problem (Padberg/Rinaldi)
  // TYPE : TSP
  // DIMENSION : 2392
  // EDGE_WEIGHT_TYPE : EUC_2D
  // NODE_COORD_SECTION
  // 1 1.63900e+03 2.15600e+03
  // 2 1.87500e+03 2.92500e+03
  // 3 2.15000e+03 2.92500e+03
  // 4 2.42500e+03 2.92500e+03
  // 5 2.52500e+03 2.67500e+03

  // specification section
  const int dim = 4;

  tsp_file << "NAME : test\n";
  tsp_file << "TYPE : TSP\n";
  tsp_file << "DIMENSION : " << dim << "\n";
  tsp_file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  tsp_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  tsp_file << "EDGE_WEIGHT_SECTION\n";

  // for (int i = 0; i < dim; ++i) {
  //   for (int j = 0; j < i; ++j) {
  //     int xi = i;
  //     int xj = j;
  //     if (i == j)
  //       tsp_file << 9999;
  //     else
  //       tsp_file << abs(xj - xi) << " ";
  //   }
  //   if (i > 0) tsp_file << "\n";
  // }

  // data section
  tsp_file << "100\n";
  tsp_file << "141 100  \n";
  tsp_file << "100 141 100 \n";

  tsp_file << "EOF";
  tsp_file.close();

  // read the tour
  ifstream res_file("/home/boboyu/workspaces/plan_ws/src/fast_planner/third_party/"
                    "LKH-2.0.9/test.txt");
  string res;
  vector<int> tour;

  while (getline(res_file, res)) {
    if (res.compare("TOUR_SECTION") == 0) {
      while (getline(res_file, res)) {
        int id = stoi(res);
        if (id == -1) break;
        tour.push_back(id);
      }
      break;
    }
  }
  for (auto id : tour) {
    std::cout << id << std::endl;
  }

  return 1;
}
