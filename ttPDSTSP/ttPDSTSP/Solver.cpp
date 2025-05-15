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

    std::sort(remainingCustomers.begin(), remainingCustomers.end(),
        [this](int a, int b) {
            return instance.tau[0][a] > instance.tau[0][b];
        });

    for (int c : remainingCustomers) {
        int choseTruck = 0;
        int posBest = -1;
        int bestDrone = -1;
        double bestTime = numeric_limits<double>::infinity();

        // --- 1. Best insertion cho truck ---
        double minTimeIncrease = numeric_limits<double>::infinity();
        double bestTruckCandidateTime = numeric_limits<double>::infinity();
        double maxDroneTime = 0.0;
        for (const auto& drone : drones) {
            maxDroneTime = max(maxDroneTime, drone.total_time);
        }

        for (int j = 1; j < truckRoute.size(); ++j) {
            double timeIncrease = tinhTimeTruckTang(truckRoute, instance.tau, c, j);
            if (timeIncrease < minTimeIncrease) {
                minTimeIncrease = timeIncrease;
                posBest = j;
            }
        }

        if (posBest != -1) {
            bestTruckCandidateTime = totalTimeTruck + minTimeIncrease;
            double totalTruckCandidateTime = max(bestTruckCandidateTime, maxDroneTime);

            bestTime = totalTruckCandidateTime;
            choseTruck = 1;
        }

        // --- 2. So sánh với drone ---
        if (find(instance.Cprime.begin(), instance.Cprime.end(), c) != instance.Cprime.end()) {
            double sumDroneTime = 0.0;
            for (const auto& d : drones) {
                sumDroneTime += d.total_time;
            }
            double meanTotalTimeDrone = (drones.size() > 0) ? (sumDroneTime / drones.size()) : 0.0;

            for (int k = 0; k < drones.size(); ++k) {
                double droneTimeIncrease = instance.tauprime[0][c] * 2;
                double candidateDroneTime = drones[k].total_time + droneTimeIncrease;

                double maxDroneTimeNow = 0.0;
                for (int d = 0; d < drones.size(); ++d) {
                    if (d == k)
                        maxDroneTimeNow = max(maxDroneTimeNow, candidateDroneTime);
                    else
                        maxDroneTimeNow = max(maxDroneTimeNow, drones[d].total_time);
                }

                double totalCandidateTime = max(totalTimeTruck, maxDroneTimeNow);

                if (totalCandidateTime < bestTime) {
                    bestTime = totalCandidateTime;
                    bestDrone = k;
                    choseTruck = 0;
                }
                else if (totalCandidateTime == bestTime &&
                    bestTruckCandidateTime == totalCandidateTime &&
                    bestTruckCandidateTime > meanTotalTimeDrone) {
                    bestDrone = k;
                    choseTruck = 0;
                }
            }
        }

        // --- 3. Cập nhật kết quả ---
        if (choseTruck == 0) {
            drones[bestDrone].route.push_back(c);
            drones[bestDrone].total_time += instance.tauprime[0][c] * 2;
        }
        else {
            totalTimeTruck += minTimeIncrease;
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
