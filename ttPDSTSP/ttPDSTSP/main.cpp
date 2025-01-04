#include <iostream>
#include "Solver.h"

using namespace std;

int main() {
    string filename = "20140813T111604";
    INSTANCE instance;
    instance.loadFromFile(filename);
    //instance.displayData();
    Solver solver(instance);
    solver.solve();
    solver.displaySolution();
    return 0;
}