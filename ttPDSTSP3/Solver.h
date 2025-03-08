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

class Solver {
public:
    Solver(const INSTANCE& instance);
    void solveA1();
    void solveA2();
    void solveA3();

    void displaySolution() const;

    // Tính thời gian tăng thêm khi thêm khách hàng vào tuyến xe tải
    double tinhTimeTruckTang(const std::vector<int>& route, const std::vector<std::vector<double>>& tau, int node, int insertPos) const;

    // Tính tổng thời gian hoàn thành của xe tải (CT)
    double tinhTotalTimeTruck(const std::vector<int>& route, const std::vector<std::vector<double>>& tau) const;

    // Tính tổng thời gian hoàn thành của drone (CD)
    double tinhMaxTimeDrones(const std::vector<Drones>& drones) const;

    // Tính tổng thời gian hoàn thành (CT và CD)
    double tinhTotaltime(const std::vector<Drones>& drones, double totalTimeTruck) const;
    vector<int> A2opt(const vector<int>& route);
    void RVNS_T();
    void RVNS_P();


    const INSTANCE& instance;
    std::vector<Drones> drones;
    std::vector<int> truckRoute;  
    double totalTimeTruck = 0;
};


#endif // SOLVER_H
