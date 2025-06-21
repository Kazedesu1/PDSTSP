#include <iostream>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <random>
#include "Solver.h"

using namespace std;

Solver::Solver(const INSTANCE& inst) : instance(inst) {}

void Solver::solveA1() {
    drones.resize(instance.UAVs);
    vector<int> remainingCustomers = instance.C;

    vector<int> truckCustomers = instance.truckonly;
    vector<int> H1 = instance.Cprime;


    // Tạo tuyến Truck ban đầu bằng nearest neighbor
    if (!truckCustomers.empty()) {
        truckRoute = { 0 };

        while (!truckCustomers.empty()) {
            int last = truckRoute.back();
            auto nearestIt = min_element(truckCustomers.begin(), truckCustomers.end(),
                [&](int a, int b) {
                    return instance.tau[last][a] < instance.tau[last][b];
                });

            truckRoute.push_back(*nearestIt);
            truckCustomers.erase(nearestIt);
        }
        truckRoute.push_back(0);

        // Tính tổng thời gian Truck 
        totalTimeTruck = tinhTotalTimeTruck(truckRoute, instance.tau);
    }
    else {
        truckRoute = { 0, 0 }; 
		totalTimeTruck = 0;
    }
    //Tính CDtb
    double CDtb = 0;
    for (int c : H1) {
        CDtb += instance.tauprime[0][c] * 2;
    }
    CDtb /= instance.UAVs;

    // Lặp lại cho đến khi CT >= CDtb
    while (totalTimeTruck < CDtb && !H1.empty()) {
        int bestCustomer = -1;
        int bestPos = -1;
        double bestCT = numeric_limits<double>::infinity();

        for (int c : H1) {
            for (int j = 1; j < truckRoute.size(); ++j) {
                double deltaCT = tinhTimeTruckTang(truckRoute, instance.tau, c, j);
                double candidateCT = totalTimeTruck + deltaCT;

                if (candidateCT < bestCT) {
                    bestCT = candidateCT;
                    bestCustomer = c;
                    bestPos = j;
                }
            }
        }

        if (bestCustomer != -1) {
            totalTimeTruck += tinhTimeTruckTang(truckRoute, instance.tau, bestCustomer, bestPos);
            truckRoute.insert(truckRoute.begin() + bestPos, bestCustomer);
            H1.erase(remove(H1.begin(), H1.end(), bestCustomer), H1.end());

            // Cập nhật CDtb
            CDtb = 0;
            for (int c : H1) {
                CDtb += instance.tauprime[0][c] * 2;
            }
            CDtb /= instance.UAVs;
        }
    }

    applydrone(H1);
}

void Solver::solveA2() {
    drones.resize(instance.UAVs);
    vector<int> H1 = instance.Cprime;  // H1 = VD
    vector<int> H2 = instance.truckonly;
    double CD_avg = 0;

    for (int c : H1) {
        CD_avg += instance.tauprime[0][c] * 2;
    }
    CD_avg /= instance.UAVs;

    if (H1.size() != instance.C.size()) {  // VD != V \ {0}
        truckRoute = { 0 };

        // Tạo tuyến Truck bằng Nearest Neighbor

        while (!H2.empty()) {
            int last = truckRoute.back();
            auto nearestIt = min_element(H2.begin(), H2.end(),
                [&](int a, int b) { return instance.tau[last][a] < instance.tau[last][b]; });
            truckRoute.push_back(*nearestIt);
            H2.erase(nearestIt);
        }
        truckRoute.push_back(0);

        totalTimeTruck = tinhTotalTimeTruck(truckRoute, instance.tau);
    }
    else {
        truckRoute = { 0,0 };
    }
    // (2) Nếu CDtb > CT, đưa node của H1 sang H2
    if (CD_avg > totalTimeTruck) {
        vector<pair<int, double>> distList;

        // sắp xếp H1 theo thứ tự giảm dần của Dist(i) là tổng của 5 khoảng cách của đỉnh i đến 5 đỉnh có khoảng cách gần đỉnh i nhất  
        for (int c : H1) {
            vector<double> distances;
            for (int i = 0; i < instance.nodes.size(); ++i) {
                if (i != c) {
                    distances.push_back(instance.tau[c][i]);
                }
            }
            sort(distances.begin(), distances.end());
            double distSum = 0;
            for (int i = 0; i < min(5, (int)distances.size()); ++i) {
                distSum += distances[i];
            }
            distList.emplace_back(c, distSum);
        }
        sort(distList.begin(), distList.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
            return a.second > b.second;
            });

        H1.clear();
        for (const auto& p : distList) {
            H1.push_back(p.first);
        }

        // Chuyển gamma * H1 khách hàng từ cuối danh sách H1 sang H2
        double gamma = 0.3;  // Hằng số gamma
        int numMove = max(1, (int)(gamma * H1.size()));

        for (int i = distList.size() - numMove; i < distList.size(); i++) {
            H2.push_back(distList[i].first);
        }

        for (int c : H2) {
            H1.erase(remove(H1.begin(), H1.end(), c), H1.end());
        }

        // Chèn khách hàng từ H2 vào Truck theo quy tắc chi phí tối thiểu
        for (int c : H2) {
            int bestPos = -1;
            double bestCT = numeric_limits<double>::infinity();

            for (int j = 1; j < truckRoute.size(); ++j) {
                double deltaCT = tinhTimeTruckTang(truckRoute, instance.tau, c, j);
                double candidateCT = totalTimeTruck + deltaCT;

                if (candidateCT < bestCT) {
                    bestCT = candidateCT;
                    bestPos = j;
                }
            }

            if (bestPos != -1) {
                totalTimeTruck += tinhTimeTruckTang(truckRoute, instance.tau, c, bestPos);
                truckRoute.insert(truckRoute.begin() + bestPos, c);
            }
        }

    }
    // Chạy tiếp bước (2) trong (2) của A1 

    //Tính CDtb
    double CDtb = 0;
    for (int c : H1) {
        CDtb += instance.tauprime[0][c] * 2;
    }
    CDtb /= instance.UAVs;

    // Lặp lại cho đến khi CT >= CDtb
    while (totalTimeTruck < CDtb && !H1.empty()) {
        int bestCustomer = -1;
        int bestPos = -1;
        double bestCT = numeric_limits<double>::infinity();

        for (int c : H1) {
            for (int j = 1; j < truckRoute.size(); ++j) {
                double deltaCT = tinhTimeTruckTang(truckRoute, instance.tau, c, j);
                double candidateCT = totalTimeTruck + deltaCT;

                if (candidateCT < bestCT) {
                    bestCT = candidateCT;
                    bestCustomer = c;
                    bestPos = j;
                }
            }
        }

        if (bestCustomer != -1) {
            totalTimeTruck += tinhTimeTruckTang(truckRoute, instance.tau, bestCustomer, bestPos);
            truckRoute.insert(truckRoute.begin() + bestPos, bestCustomer);
            H1.erase(remove(H1.begin(), H1.end(), bestCustomer), H1.end());

            // Cập nhật CDtb
            CDtb = 0;
            for (int c : H1) {
                CDtb += instance.tauprime[0][c] * 2;
            }
            CDtb /= instance.UAVs;
        }
    }
    applydrone(H1);
}


void Solver::solveA3() {
    // (1) Khởi tạo tuyến Truck bằng 5-nearest neighbor
    drones.resize(instance.UAVs);
    vector<int> H2 = instance.C;   // H2 = V \ {0}
    vector<int> H1;

    // Dùng 5-nearest neighbor để xây dựng tuyến Truck ban đầu
    truckRoute = { 0 };
    while (!H2.empty()) {
        int last = truckRoute.back();

        // tạo danh sách 5 khách hàng gần nhất
        vector<int> nearest;
        vector<int> candidates = H2;  // Copy danh sách khách hàng từ H2

        for (int i = 0; i < min(5, (int)candidates.size()); i++) {
            auto nearestIt = min_element(candidates.begin(), candidates.end(),
                [&](int a, int b) { return instance.tau[last][a] < instance.tau[last][b]; });

            nearest.push_back(*nearestIt);
            candidates.erase(nearestIt);  // Xóa khỏi danh sách tạm thời
        }

        //  Chọn khách hàng gần nhất trong danh sách 5-nearest
        if (!nearest.empty()) {
            int nextCustomer = nearest[0];  // Chọn khách hàng đầu tiên
            truckRoute.push_back(nextCustomer);
            H2.erase(remove(H2.begin(), H2.end(), nextCustomer), H2.end());  // Xóa khách hàng khỏi H2
        }
    }
    truckRoute.push_back(0);

    totalTimeTruck = tinhTotalTimeTruck(truckRoute, instance.tau);


    // (2) Di chuyển khách hàng từ Truck sang Drone để cân bằng tải
    double CDtb = 0;
	vector<int> droneeligible = instance.Cprime;  // Khách hàng có thể giao bằng Drone 
    while (totalTimeTruck > CDtb && !droneeligible.empty()) {
        int bestCustomer = -1;
        double maxDelta = -numeric_limits<double>::infinity();
        double bestdeltaCT = 0;

        // Tìm khách hàng drone có thể chuyển có giá trị deltaCT - tD_i lớn nhất

        for (int c : droneeligible) {
            auto it = find(truckRoute.begin(), truckRoute.end(), c);
            if (it == truckRoute.end()) continue;

            int pos = distance(truckRoute.begin(), it);

            // Tính deltaCT nếu loại bỏ c khỏi Truck
            double deltaCT = instance.tau[truckRoute[pos - 1]][c] + instance.tau[c][truckRoute[pos + 1]] - instance.tau[truckRoute[pos - 1]][truckRoute[pos + 1]];;


            //  Tính giá trị deltaCTCT - tD_i
            double tD_i = instance.tauprime[0][c] * 2;
            double score = deltaCT - tD_i;

            // Chọn khách hàng có giá trị lớn nhất
            if (score > maxDelta) {
                maxDelta = score;
                bestCustomer = c;
                bestdeltaCT = deltaCT;
            }
        }

        if (bestCustomer == -1) break;

        // Di chuyển khách hàng từ Truck sang Drone
        H1.push_back(bestCustomer);

        droneeligible.erase(remove(droneeligible.begin(), droneeligible.end(), bestCustomer), droneeligible.end());
        truckRoute.erase(remove(truckRoute.begin(), truckRoute.end(), bestCustomer), truckRoute.end());
        totalTimeTruck -= bestdeltaCT;
        // tính CDtb 
        CDtb = 0;
        for (int c : H1) {
            CDtb += instance.tauprime[0][c] * 2;
        }
        CDtb /= max(1, (int)drones.size());
    }

    applydrone(H1);
}



double Solver::tinhTotalTimeTruck(const vector<int>& route, const vector<vector<double>>& tau) const {
    double total = 0.0;
    for (size_t i = 1; i < route.size(); ++i) {
        total += tau[route[i - 1]][route[i]];
    }
    return total;
}

double Solver::tinhMaxTimeDrones(const vector<Drones>& drones) const {
    double maxDroneTime = 0.0;
    for (const auto& drone : drones) {
        maxDroneTime = max(maxDroneTime, drone.total_time);
    }
    return maxDroneTime;
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
    if (insertPos == route.size()) {
        return tau[route.back()][node] + tau[node][0] - tau[route.back()][0];
    }
    else {
        int prevNode = route[insertPos - 1];
        int nextNode = route[insertPos];
        return tau[prevNode][node] + tau[node][nextNode] - tau[prevNode][nextNode];
    }
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

void Solver::applydrone(vector<int>& H1) {
    sort(H1.begin(), H1.end(), [&](int a, int b) {
        return instance.tau[0][a] > instance.tau[0][b];
        });

    for (int c : H1) {
        int bestDrone = 0;
        double minCD = numeric_limits<double>::infinity();

        for (int k = 0; k < drones.size(); ++k) {
            double candidateCD = drones[k].total_time;
            if (candidateCD < minCD) {
                minCD = candidateCD;
                bestDrone = k;
            }
        }

        drones[bestDrone].route.push_back(c);
        drones[bestDrone].total_time += instance.tauprime[0][c] * 2;
    }
}




//void Solver::RVNS_T() {
//    int k_max = 2;
//    int iterTruck = 10;
//    int iter1 = 0;
//
//    // Nếu tuyến Truck quá ngắn, không tối ưu hóa
//    if (truckRoute.size() <= 3) return;
//
//    vector<int> bestRoute = truckRoute;
//    double bestTime = totalTimeTruck;
//
//    while (iter1 < iterTruck) {
//        int k = 1;
//
//        while (k <= k_max) {
//            vector<int> newRoute = bestRoute;
//            double newTime = bestTime;
//
//            if (k == 1) {
//                // N1: Swap hai khách hàng bất kỳ trong truckRoute
//                if (newRoute.size() > 3) {
//                    int i, j;
//                    do {
//                        i = rand() % (newRoute.size() - 2) + 1;  // Chọn ngẫu nhiên trong khoảng [1, n-1]
//                        j = rand() % (newRoute.size() - 2) + 1;
//                    } while (i == j);
//
//                    swap(newRoute[i], newRoute[j]);
//
//                    // Áp dụng 2-opt sau khi swap
//                    newRoute = A2opt(newRoute);
//                }
//            }
//            else if (k == 2) {
//                // N2: Đảo ngược một đoạn khách hàng trong truckRoute**
//                if (newRoute.size() > 4) {
//                    int i, j;
//                    do {
//                        i = rand() % (newRoute.size() - 3) + 1;
//                        j = rand() % (newRoute.size() - 3) + 2;
//                    } while (i >= j);
//
//                    reverse(newRoute.begin() + i, newRoute.begin() + j);
//
//                    // Áp dụng 2-opt sau khi đảo ngược đoạn đường
//                    newRoute = A2opt(newRoute);
//                }
//            }
//
//            // Tính lại tổng thời gian Truck mới
//            newTime = tinhTotalTimeTruck(newRoute, instance.tau);
//
//            // Nếu tuyến đường mới tốt hơn, cập nhật lời giải tốt nhất
//            if (newTime < bestTime) {
//                bestRoute = newRoute;
//                bestTime = newTime;
//                k = 1;
//                iter1 = 0;
//            }
//            else {
//                k++;
//            }
//        }
//
//        iter1++;
//    }
//
//    truckRoute = bestRoute;
//    totalTimeTruck = bestTime;
//}
//
//
//vector<int> Solver::A2opt(const vector<int>& route) {
//    vector<int> newRoute = route;
//    bool improved = true;
//
//    while (improved) {
//        improved = false;
//        for (size_t i = 1; i < newRoute.size() - 2; ++i) {
//            for (size_t j = i + 1; j < newRoute.size() - 1; ++j) {
//                double oldCost = instance.tau[newRoute[i - 1]][newRoute[i]] +
//                    instance.tau[newRoute[j]][newRoute[j + 1]];
//
//                double newCost = instance.tau[newRoute[i - 1]][newRoute[j]] +
//                    instance.tau[newRoute[i]][newRoute[j + 1]];
//
//                if (newCost < oldCost) {
//                    reverse(newRoute.begin() + i, newRoute.begin() + j + 1);
//                    improved = true;
//                }
//            }
//        }
//    }
//
//    return newRoute;
//}
//
//void Solver::RVNS_P() {
//    int iterDrone = int(instance.n/4);
//    int iter3 = 0;
//
//    if (drones.empty()) return;
//
//    // Nếu tổng số khách hàng trong drone <= 1, không tối ưu
//    int sumnode = 0;
//    for (const auto& drone : drones) {
//        sumnode += drone.route.size();
//    }
//    if (sumnode <= 1) return;
//
//    while (iter3 < iterDrone) {
//        bool improved = false;
//        int k = 1;
//        int k_max = 4;
//
//        while (k <= k_max) {
//            double oldCD = tinhMaxTimeDrones(drones);
//            bool localImproved = false;
//            vector<Drones> newRoute = drones;
//
//            if (k == 1) {
//                //  Np1: Relocate khách hàng từ drone có thời gian lớn nhất sang drone có thời gian nhỏ nhất
//                int maxDrone = -1, minDrone = -1;
//                double maxTime = -1, minTime = numeric_limits<double>::infinity();
//
//                for (size_t d = 0; d < drones.size(); ++d) {
//                    if (drones[d].route.size() > 1) {
//                        if (drones[d].total_time > maxTime) {
//                            maxTime = drones[d].total_time;
//                            maxDrone = d;
//                        }
//                        if (drones[d].total_time < minTime) {
//                            minTime = drones[d].total_time;
//                            minDrone = d;
//                        }
//                    }
//                }
//
//                if (maxDrone != -1 && minDrone != -1 && !newRoute[maxDrone].route.empty()) {
//                    int idx = rand() % newRoute[maxDrone].route.size();
//                    int customer = newRoute[maxDrone].route[idx];
//                    newRoute[maxDrone].route.erase(newRoute[maxDrone].route.begin() + idx);
//                    newRoute[minDrone].route.push_back(customer);
//                    localImproved = true;
//                }
//            }
//            else if (k == 2) {
//                // Np2: Hoán đổi khách hàng giữa hai drone khác nhau
//                if (drones.size() > 1) {
//                    int d1, d2;
//                    do {
//                        d1 = rand() % drones.size();
//                        d2 = rand() % drones.size();
//                    } while (d1 == d2 || drones[d1].route.empty() || drones[d2].route.empty());
//
//                    int idx1 = rand() % newRoute[d1].route.size();
//                    int idx2 = rand() % newRoute[d2].route.size();
//                    swap(newRoute[d1].route[idx1], newRoute[d2].route[idx2]);
//                    localImproved = true;
//                }
//            }
//
//            if (localImproved) {
//                // cập nhật lại total_time của từng drone trong newRoute
//                for (auto& drone : newRoute) {
//                    drone.total_time = 0;
//                    for (int c : drone.route) {
//                        drone.total_time += instance.tauprime[0][c] * 2;
//                    }
//                }
//
//                // Tính lại CD mới sau khi thay đổi `Np1` hoặc `Np2`
//                double newCD = tinhMaxTimeDrones(newRoute);
//                if (newCD < oldCD) {
//                    drones = newRoute;
//                    improved = true;
//                    iter3 = 0;
//                    k = 1;
//                    continue;
//                }
//            }
//
//            // Tạo danh sách PE để dùng cho Np3, Np4
//            vector<int> PE;
//            for (const auto& drone : drones) {
//                PE.insert(PE.end(), drone.route.begin(), drone.route.end());
//            }
//            vector<int> newPE = PE;
//
//            if (k == 3) {
//                //  Np3: Dịch chuyển vòng lặp một đoạn khách hàng trong PE
//                if (newPE.size() > 2) {
//                    int k1, k2;
//                    do {
//                        k1 = rand() % (newPE.size() - 2);
//                        k2 = rand() % (newPE.size() - 1);
//                    } while (k1 >= k2);
//
//                    int maxAlpha = k2 - k1;
//                    if (maxAlpha > 0) {
//                        int alpha = 1 + rand() % maxAlpha;
//                        rotate(newPE.begin() + k1, newPE.begin() + k2 - alpha, newPE.begin() + k2);
//                        localImproved = true;
//                    }
//                }
//            }
//            else if (k == 4) {
//                //  Np4: Đảo ngược thứ tự một đoạn khách hàng trong PE
//                if (newPE.size() > 2) {
//                    int k1, k2;
//                    do {
//                        k1 = rand() % (newPE.size() - 2);
//                        k2 = rand() % (newPE.size() - 1);
//                    } while (k1 >= k2);
//                    reverse(newPE.begin() + k1, newPE.begin() + k2);
//                    localImproved = true;
//                }
//            }
//
//            if (localImproved) {
//                //  Tính lại CD mới sau khi thay đổi PE
//                double newCD = 0;
//                for (auto& drone : drones) {
//                    drone.total_time = 0;
//                    for (int c : drone.route) {
//                        drone.total_time += instance.tauprime[0][c] * 2;
//                    }
//                    newCD = max(newCD, drone.total_time);
//                }
//
//                if (newCD < oldCD) {
//                    PE = newPE;
//                    improved = true;
//                    iter3 = 0;
//                    k = 1;
//
//                    // Cập nhật lại danh sách khách hàng trong drones theo PE
//                    size_t index = 0;
//                    for (auto& drone : drones) {
//                        drone.route.clear();
//                    }
//                    for (size_t i = 0; i < PE.size(); ++i) {
//                        int droneIdx = i % drones.size();
//                        drones[droneIdx].route.push_back(PE[i]);
//                    }
//                    continue;
//                }
//            }
//
//            k++;  // Chuyển sang hàng xóm tiếp theo nếu không có cải thiện
//        }
//
//        if (!improved) iter3++;  // Nếu không có cải thiện, tăng iter3
//    }
//}
