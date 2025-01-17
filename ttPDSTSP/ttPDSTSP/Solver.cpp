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

    for (int c : remainingCustomers) {
        int choseTruck = 0;
        int posBest = -1;  
        int bestDrone = -1; 
        double bestTime = numeric_limits<double>::infinity();

        for (int j = 1; j < truckRoute.size(); ++j) {
            double timeIncrease = tinhTimeTruckTang(truckRoute, instance.tau, c, j);
            double candidateTruckTime = totalTimeTruck + timeIncrease;

            double maxDroneTime = 0.0;
            for (const auto& drone : drones) {
                maxDroneTime = max(maxDroneTime, drone.total_time);
            }

            double totalCandidateTime = max(candidateTruckTime, maxDroneTime);

            if (totalCandidateTime < bestTime) {
                bestTime = totalCandidateTime;
                posBest = j;
				choseTruck = 1;  
            }
        }

        if (find(instance.Cprime.begin(), instance.Cprime.end(), c) != instance.Cprime.end()) {
            for (int k = 0; k < drones.size(); ++k) {
                double droneTimeIncrease = instance.tauprime[0][c] * 2;
                double candidateDroneTime = drones[k].total_time + droneTimeIncrease;

                double maxDroneTime = 0.0;
                for (const auto& drone : drones) {
                    maxDroneTime = max(maxDroneTime, drone.total_time);
                }

                double totalCandidateTime = max({ totalTimeTruck, candidateDroneTime, maxDroneTime });

                if (totalCandidateTime <= bestTime) {
                    bestTime = totalCandidateTime;
                    bestDrone = k;  
					choseTruck = 0;
                }
            }
        }

        if (choseTruck == 0) {
            drones[bestDrone].route.push_back(c);
            drones[bestDrone].total_time += instance.tauprime[0][c] * 2;
        }
        else if (choseTruck == 1) {
            totalTimeTruck += tinhTimeTruckTang(truckRoute, instance.tau, c, posBest);
            truckRoute.insert(truckRoute.begin() + posBest, c);
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
