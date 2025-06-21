#include "Instances.h"
#include <stdexcept>
using namespace std;

bool INSTANCE::loadFromFile(const string& filename)
{
    try {
        // Đọc tau.csv
        ifstream file(filename + "/tau.csv");
        if (!file.is_open()) {
            throw runtime_error("Unable to open tau.csv file.");
        }
        string line;
        while (getline(file, line)) {
            vector<double> row;
            stringstream ss(line);
            string value;
            while (getline(ss, value, ',')) {
                row.push_back(stod(value));
            }
            tau.push_back(row);
        }
        file.close();

        // Đọc tauprime.csv
        ifstream file2(filename + "/tauprime.csv");
        if (!file2.is_open()) {
            throw runtime_error("Unable to open tauprime.csv file.");
        }
        string line2;
        while (getline(file2, line2)) {
            vector<double> row2;
            stringstream ss2(line2);
            string value2;
            while (getline(ss2, value2, ',')) {
                row2.push_back(stod(value2));
            }
            tauprime.push_back(row2);
        }
        file2.close();

        //// Đọc Cprime.csv
        ifstream file3(filename + "/Cprime.csv");
        if (!file3.is_open()) {
            throw runtime_error("Unable to open Cprime.csv file.");
        }
        string line3;
        while (getline(file3, line3)) {
            stringstream ss3(line3);
            string value3;
            while (getline(ss3, value3, ',')) {
                Cprime.push_back(stod(value3));
            }
        }
        file3.close();

        // Đọc Nodes.csv
        ifstream file4(filename + "/nodes.csv");
        if (!file4.is_open()) {
            throw runtime_error("Unable to open tauprime.csv file.");
        }
        string line4;
        while (getline(file4, line4)) {
            vector<double> row4;
            stringstream ss4(line4);
            string value4;
            while (getline(ss4, value4, ',')) {
                row4.push_back(stod(value4));
            }
            nodes.push_back(row4);
        }
        file4.close();

        UAVs = 1;
        n = tau.size(); // n = c+2

        // Tính tg drone đi từ 0 đến node i và quay về
        dDrones.resize(n);
        for (int i = 0; i < n; i++)
        {
            dDrones[i] = tinhkc(nodes, i) * 2 / nodes[0][3];
        }

        for (int idx = Cprime.size() - 1; idx >= 0; idx--) {
            if ((tauprime[0][Cprime[idx]]) > 15) {
                Cprime.erase(Cprime.begin() + idx);
            }
        }

        for (int i = 1; i < n - 1; i++) {
            if (find(Cprime.begin(), Cprime.end(), i) == Cprime.end()) {
                truckonly.push_back(i);
            }
            C.push_back(i);
        }
        return true;
    }

    catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return false;
    }
}
void INSTANCE::displayData()
{
    cout << "tau: " << endl;
    for (const auto& row : tau)
    {
        for (const auto& value : row)
        {
            cout << value << " ";
        }
        cout << endl;
    }
    cout << "tauprime: " << endl;
    for (const auto& row : tauprime)
    {
        for (const auto& value : row)
        {
            cout << value << " ";
        }
        cout << endl;
    }
    cout << "Cprime: ";
    for (const auto& value : Cprime)
    {
        cout << value << " ";
    }
    cout << endl << "nodes: " << endl;
    for (const auto& row : nodes)
    {
        for (const auto& value : row)
        {
            cout << value << " ";
        }
        cout << endl;
    }
    cout << "Ddrone: ";
    for (const auto& value : dDrones)
    {
        cout << value << " ";
    }
    cout << endl;
    cout << "UAVs: " << UAVs << endl;
    cout << "n: " << n << endl;
}

double tinhkc(const vector<vector<double>>& nodes, int node) {
    double c = 0;
    c = sqrt(pow(nodes[0][1] - nodes[node][1], 2) + pow(nodes[0][2] - nodes[node][2], 2));
    return c;
}
