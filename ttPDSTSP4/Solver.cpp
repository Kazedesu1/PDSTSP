#include "Solver.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <unordered_set>
using namespace std;

Solver::Solver(const INSTANCE& inst) : instance(inst) {}

void Solver::solve() {
	drones.resize(instance.UAVs);
    vector<int> truckCustomers = instance.truckonly;
	vector<int> remainingCustomers = instance.Cprime;
    // Insert truck-only customers
    while (!truckCustomers.empty()) {
        double minCost = numeric_limits<double>::infinity();
        int bestPos = -1;
        int bestCustomer = -1;

        for (int customer : truckCustomers) {
            for (int i = 1; i < truckRoute.size(); ++i) {
                int prev = truckRoute[i - 1];
                int next = truckRoute[i];
                double cost = instance.tau[prev][customer] + instance.tau[customer][next] - instance.tau[prev][next];

                if (cost < minCost) {
                    minCost = cost;
                    bestCustomer = customer;
                    bestPos = i;
                }
            }
        }

        if (bestCustomer != -1 && bestPos != -1) {
            truckRoute.insert(truckRoute.begin() + bestPos, bestCustomer);
            truckCustomers.erase(remove(truckCustomers.begin(), truckCustomers.end(), bestCustomer), truckCustomers.end());
        }
        else {
            throw string("Insertion failed in Cheapest Insertion.");
        }
    }

    totalTimeTruck = tinhTotalTimeTruck(truckRoute, instance.tau);

    // ------------------ Greedy Distance-Density Balance ------------------
    double T_budget = 1.2*totalTimeTruck;
    double total_ti = 0.0;
    for (int c : instance.Cprime)
        total_ti += instance.tauprime[0][c] * 2;
    double avg_ti = total_ti / instance.Cprime.size();
    int m = min((int)instance.Cprime.size(), (int)((T_budget / avg_ti) * drones.size()));

    double alpha = 0.7;  

    // Tính score kết hợp distance và density
    vector<pair<int, double>> scoreList;
    for (int c : instance.Cprime) {
        // Tính distance
        double dist = instance.tauprime[0][c];

        // Tính density: tổng khoảng cách đến 5 láng giềng gần nhất
        vector<double> dists;
        for (int cc : instance.Cprime) {
            if (cc != c)
                dists.push_back(instance.tau[c][cc]);
        }
        sort(dists.begin(), dists.end());
        double density = 0;
        for (int i = 0; i < min(5, (int)dists.size()); ++i)
            density += dists[i];

        double score = alpha * dist + (1 - alpha) * density;
        scoreList.emplace_back(c, score);
    }

    // Chọn m khách có score nhỏ nhất để giao bằng drone
    sort(scoreList.begin(), scoreList.end(), [](auto& a, auto& b) {
        return a.second < b.second;
        });

    unordered_set<int> droneSet;
    for (int i = 0; i < m; ++i)
        droneSet.insert(scoreList[i].first);

    // Phân phối
    for (int c : remainingCustomers) {
        if (droneSet.count(c)) {
            int bestDrone = -1;
            double minTime = numeric_limits<double>::infinity();
            for (int i = 0; i < drones.size(); ++i) {
                double t = drones[i].total_time + instance.tauprime[0][c] * 2;
                if (t < minTime) {
                    minTime = t;
                    bestDrone = i;
                }
            }
            if (bestDrone != -1) {
                drones[bestDrone].route.push_back(c);
                drones[bestDrone].total_time += instance.tauprime[0][c] * 2;
            }
        }
        else {
            int bestPos = -1;
            double minCost = numeric_limits<double>::infinity();
            for (int i = 1; i < truckRoute.size(); ++i) {
                double cost = tinhTimeTruckTang(truckRoute, instance.tau, c, i);
                if (cost < minCost) {
                    minCost = cost;
                    bestPos = i;
                }
            }
            if (bestPos != -1) {
                truckRoute.insert(truckRoute.begin() + bestPos, c);
                totalTimeTruck += minCost;
            }
            else {
                throw string("Failed to insert into truck route.");
            }
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
double Solver::tinhTotalTimeTruck(const vector<int>& route, const vector<vector<double>>& tau) const {
    double total = 0.0;
    for (size_t i = 1; i < route.size(); ++i) {
        total += tau[route[i - 1]][route[i]];
    }
    return total;
}