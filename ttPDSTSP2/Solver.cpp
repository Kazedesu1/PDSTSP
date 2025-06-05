#include "Solver.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <ctime>
using namespace std;

Solver::Solver(const INSTANCE& inst) : instance(inst) {}

void Solver::solve() {
    drones.resize(instance.UAVs);
    vector<int> remainingCustomers = instance.C;
    // Sort remainingCustomers by distance from the depot in descending order
    std::sort(remainingCustomers.begin(), remainingCustomers.end(), 
       [&](int a, int b) {
           return instance.tau[0][a] > instance.tau[0][b];
       });
    
    for (int c : remainingCustomers) {
        int p = -1;
        double bestCost = numeric_limits<double>::infinity();
        bool to_drone = false;

        vector<int> position;
        for (int i = 1; i < truckRoute.size(); ++i)
            position.push_back(i);

        for (int i : position) {
            double Tnew = totalTimeTruck + tinhTimeTruckTang(truckRoute, instance.tau, c, i);
            if (Tnew <= bestCost + 0.00001) {
                to_drone = false;
                bestCost = Tnew;
                p = i;
            }
        }

        if (find(instance.Cprime.begin(), instance.Cprime.end(), c) != instance.Cprime.end()) {
            for (int i = 0; i < drones.size(); ++i) {
                double Tnew = drones[i].total_time + instance.tauprime[0][c] * 2;
                if (Tnew < bestCost) {
                    to_drone = true;
                    bestCost = Tnew;
                    p = i;
                }
            }
        }

        if (p == -1) throw string("SHIT HAPPEN IN INSERTION!");

        if (to_drone) {
            drones[p].route.push_back(c);
            drones[p].total_time += instance.tauprime[0][c] * 2;
        }
        else {
            totalTimeTruck += tinhTimeTruckTang(truckRoute, instance.tau, c, p);
            truckRoute.insert(truckRoute.begin() + p, c);
        }
    }
}



double Solver::tinhTotaltime(const vector<Drones>& drones, double totalTimeTruck) const {
    double maxDroneTime = 0.0;
    for (const auto& drone : drones) {
        if (drone.total_time > maxDroneTime) {
            maxDroneTime = drone.total_time;
        }
    }
    return max(maxDroneTime, totalTimeTruck);
}

double Solver::tinhTimeTruckTang(const vector<int>& route, const vector<vector<double>>& tau, int node, int insertPos) const {
    int prevNode = route[insertPos - 1];
    int nextNode = route[insertPos];

    // Tính thời gian tăng thêm khi chèn nút
    double timeIncrease = tau[prevNode][node] + tau[node][nextNode] - tau[prevNode][nextNode];
    return timeIncrease;
}

void Solver::displaySolution() const {
    cout << "\nSolution:\n";
    // Hiển thị lộ trình của xe tải
    cout << "Truck Route: ";
    for (size_t i = 0; i < truckRoute.size(); ++i) {
        cout << truckRoute[i];
        if (i < truckRoute.size() - 1) {
            cout << " ";
        }
    }
    cout << "\nTotal Time Truck: " << totalTimeTruck << "\n";

    // Hiển thị lộ trình của các drone
    for (size_t i = 0; i < drones.size(); ++i) {
        cout << "Drone " << i + 1 << " Route: ";
        for (size_t j = 0; j < drones[i].route.size(); ++j) {
            cout << drones[i].route[j];
            if (j < drones[i].route.size() - 1) {
                cout << " ";
            }
        }
        cout << "\nTotal Time Drone " << i + 1 << ": " << drones[i].total_time << "\n";
    }

    double maxTotalTime = tinhTotaltime(drones, totalTimeTruck);
    cout << "\nMaximum Total Time (Truck + Drones): " << maxTotalTime << "\n";
}
