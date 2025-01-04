#include "Solver.h"
#include <iostream>
#include <algorithm>
#include <limits>
using namespace std;

Solver::Solver(const INSTANCE& inst) : instance(inst) {}

void Solver::solve() {
    drones.resize(instance.UAVs);
    vector<int> remainingCustomers = instance.C;

    while (!remainingCustomers.empty()) {
        int bestTruckNode = -1;
        double minTimeTruck = numeric_limits<double>::infinity();
        int bestTruckPos = -1;

        double temptimetruck;
        double Totaltime;
        for (int node : remainingCustomers) {
            for (int j = 1; j < truckRoute.size(); ++j) {

                temptimetruck = totalTimeTruck;
                temptimetruck += tinhTimeTruckTang(truckRoute, instance.tau, node, j);
                Totaltime = -1;
                for (Drones drone : drones) {
                    if (drone.total_time >= Totaltime) {
                        Totaltime = drone.total_time;
                    }
                }
                if (temptimetruck >= Totaltime) {
                    Totaltime = temptimetruck;
                }

                if (Totaltime < minTimeTruck) {
                    minTimeTruck = Totaltime;
                    bestTruckNode = node;
                    bestTruckPos = j;
                }
            }
        }

        int bestDrone = -1;
        int bestDroneNode = -1;
        double minTimeDrone = numeric_limits<double>::infinity();

        double temptimedrone;
        for (int i = 0; i < drones.size(); ++i) {
            for (int node : remainingCustomers) {
                if (find(instance.Cprime.begin(), instance.Cprime.end(), node) != instance.Cprime.end()) {

                    temptimedrone = drones[i].total_time;
                    temptimedrone += instance.dDrones[node];
                    Totaltime = -1;
                    for (Drones drone : drones) {
                        if (drone.total_time >= Totaltime) {
                            Totaltime = drone.total_time;
                        }
                    }
                    if (temptimedrone >= Totaltime) {
                        Totaltime = temptimedrone;
                    }
                    if (totalTimeTruck >= Totaltime) {
                        Totaltime = totalTimeTruck;
                    }

                    if (Totaltime < minTimeDrone ) {
                        minTimeDrone = Totaltime;
                        bestDrone = i;
                        bestDroneNode = node;
                    }
                }
            }
        }

        if (minTimeDrone <= minTimeTruck) {
            drones[bestDrone].route.push_back(bestDroneNode);
            drones[bestDrone].total_time += instance.dDrones[bestDroneNode];
            remainingCustomers.erase(remove(remainingCustomers.begin(), remainingCustomers.end(), bestDroneNode), remainingCustomers.end());
        }
        else if (bestTruckNode != -1) {
            truckRoute.insert(truckRoute.begin() + bestTruckPos, bestTruckNode);
            totalTimeTruck = minTimeTruck;
            remainingCustomers.erase(remove(remainingCustomers.begin(), remainingCustomers.end(), bestTruckNode), remainingCustomers.end());
        }
    }
}

double Solver::tinhTotaltime(const vector<Drones>& drones, double totalTimeTruck)  const {
    double maxDroneTime = 0.0;
    for (const auto& drone : drones) {
        if (drone.total_time > maxDroneTime) {
            maxDroneTime = drone.total_time;
        }
    }
    return max(maxDroneTime, totalTimeTruck);
}

double Solver::tinhTimeTruckTang(const vector<int>& route, const vector<vector<double>>& tau, int node, int insertPos ) const {
    int prevNode = route[insertPos - 1];
    int nextNode = route[insertPos];

    double totalCost = -tau[prevNode][nextNode] +
        tau[prevNode][node] +
        tau[node][nextNode];
    return totalCost;
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
