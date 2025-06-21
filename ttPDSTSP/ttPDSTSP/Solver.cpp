#include "Solver.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <unordered_map>
#include <ctime>
using namespace std;

Solver::Solver(const INSTANCE& inst) : instance(inst) {}

void Solver::solve() {
    drones.resize(instance.UAVs);
    vector<int> remainingCustomers = instance.Cprime;
    vector<int> truckCustomers = instance.truckonly;


    // === Phase 1: Cheapest Insertion cho truck-only
    while (!truckCustomers.empty()) {
        double minCost = numeric_limits<double>::infinity();
        int bestPos = -1, bestCustomer = -1;

        for (int customer : truckCustomers) {
            for (int i = 1; i < truckRoute.size(); ++i) {
                double cost = instance.tau[truckRoute[i - 1]][customer]
                    + instance.tau[customer][truckRoute[i]]
                    - instance.tau[truckRoute[i - 1]][truckRoute[i]];
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

    // === Phase 2: xử lý remainingCustomers
    unordered_map<int, InsertionInfo> bestInsertion;

    for (int c : remainingCustomers) {
        double minInc = numeric_limits<double>::infinity();
        int bestPos = -1;
        for (int i = 1; i < truckRoute.size(); ++i) {
            double inc = tinhTimeTruckTang(truckRoute, instance.tau, c, i);
            if (inc < minInc) {
                minInc = inc;
                bestPos = i;
            }
        }
        bestInsertion[c] = { minInc, bestPos };
    }

    sort(remainingCustomers.begin(), remainingCustomers.end(), [&](int a, int b) {
        return bestInsertion[a].increase > bestInsertion[b].increase;
        });

    while (!remainingCustomers.empty()) {
        int c = remainingCustomers[0];
        bool to_drone = false;
        int pos = bestInsertion[c].pos;
        double bestCost = totalTimeTruck + bestInsertion[c].increase;

        if (find(instance.Cprime.begin(), instance.Cprime.end(), c) != instance.Cprime.end()) {
            for (int i = 0; i < drones.size(); ++i) {
                double Tnew = drones[i].total_time + instance.tauprime[0][c] * 2;
                if (Tnew < bestCost) {
                    bestCost = Tnew;
                    to_drone = true;
                    pos = i;
                }
            }
        }

        if (to_drone) {
            drones[pos].route.push_back(c);
            drones[pos].total_time += instance.tauprime[0][c] * 2;
            remainingCustomers.erase(remove(remainingCustomers.begin(), remainingCustomers.end(), c), remainingCustomers.end());
            bestInsertion.erase(c);
        }
        else {
            totalTimeTruck += bestInsertion[c].increase;
            truckRoute.insert(truckRoute.begin() + pos, c);
            remainingCustomers.erase(remove(remainingCustomers.begin(), remainingCustomers.end(), c), remainingCustomers.end());
            bestInsertion.erase(c);

            int insertedPos = pos;

            for (int cust : remainingCustomers) {
                auto& info = bestInsertion[cust];
                int oldPos = info.pos;
                double oldInc = info.increase;

                bool isOverwritten = (oldPos == insertedPos);
                int newPos = oldPos;
                double newInc = oldInc;

                // (1) Shift nếu cần
                if (oldPos >= insertedPos) {
                    newPos++;
                }

                // (2) Nếu bị ghi đè -> tính lại toàn bộ
                if (isOverwritten) {
                    newInc = numeric_limits<double>::infinity();
                    newPos = -1;
                    for (int i = 1; i < truckRoute.size(); ++i) {
                        double cost = tinhTimeTruckTang(truckRoute, instance.tau, cust, i);
                        if (cost < newInc) {
                            newInc = cost;
                            newPos = i;
                        }
                    }
                }

                // (3) So sánh với 2 vị trí mới: insertedPos và insertedPos + 1
                for (int i = insertedPos; i <= insertedPos + 1 && i < truckRoute.size(); ++i) {
                    double testCost = tinhTimeTruckTang(truckRoute, instance.tau, cust, i);
                    if (testCost < newInc) {
                        newInc = testCost;
                        newPos = i;
                    }
                }

                bestInsertion[cust] = { newInc, newPos };
            }

            // Sắp xếp lại để lấy đỉnh tốt nhất tiếp theo
            sort(remainingCustomers.begin(), remainingCustomers.end(), [&](int a, int b) {
                return bestInsertion[a].increase > bestInsertion[b].increase;
                });
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