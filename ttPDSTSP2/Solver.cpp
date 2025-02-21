#include "Solver.h"
#include <iostream>
#include <algorithm>
#include "utils.h"
#include <limits>
#include <cstdlib>
#include <ctime>
using namespace std;

Solver::Solver(const INSTANCE& inst) : instance(inst) {}

void Solver::solve() {
    drones.resize(instance.UAVs);
    vector<int> remainingCustomers = instance.C;
    int blink_rate = 0.1;

    for (int c : remainingCustomers) {
        int choseTruck = 0;
        int posBest = -1;
        int bestDrone = -1;
        double bestTime = numeric_limits<double>::infinity();

        int p = -1;
        double bestCost = 1e10;
        bool to_drone = false;
        vector<int>& tour = truckRoute;

        vector<int> position;
        for (int i = 1; i < tour.size(); ++i)
            position.push_back(i);
        Utils::shuffle(position);

        for (int i : position) {
            if (Utils::real_random_generator(0, 1) >= 1 - blink_rate && p != -1) continue;

            double Tnew = totalTimeTruck + tinhTimeTruckTang(tour, instance.tau, c, i);
            if (Tnew <= bestCost + 0.00001 ||
                (Tnew - totalTimeTruck < 0.00001 && Utils::real_random_generator(0, 1) < 0.5)) {
                to_drone = false;
                bestCost = Tnew;
                p = i;
            }
        }

        if (find(instance.Cprime.begin(), instance.Cprime.end(), c) != instance.Cprime.end()) {
            for (int i = 0; i < drones.size(); ++i) {
                if (Utils::real_random_generator(0, 1) >= 1 - blink_rate && p != -1) continue;
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
