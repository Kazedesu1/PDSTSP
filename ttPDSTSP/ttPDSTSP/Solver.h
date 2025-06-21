#pragma once
#ifndef SOLVER_H
#define SOLVER_H

#include "Instances.h"
#include <vector>
#include <set>

struct Drones {
    double total_time = 0;
    std::vector<int> route;
    Drones() : total_time(0), route() {}
};
struct InsertionInfo {
    double increase;
    int pos;
};

class Solver {
public:
    Solver(const INSTANCE& instance);
    void solve();
    void displaySolution() const;
    double tinhTimeTruckTang(const vector<int>& route, const vector<vector<double>>& tau, int node, int insertPos ) const;
    double tinhTotaltime(const vector<Drones>& drones, double totalTimeTruck)  const;
    double tinhTotalTimeTruck(const vector<int>& route, const vector<vector<double>>& tau) const;
    const INSTANCE& instance;
    std::vector<Drones> drones;
    std::vector<int> truckRoute = { 0, 0 };
    double totalTimeTruck = 0;
    void ruinAndRecreate() ;
    void localSearch() ;
    void setAlpha(double a);
};

#endif // SOLVER_H
