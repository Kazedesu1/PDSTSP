#include "ilcplex/ilocplex.h" 
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept> 
using namespace std;

int main() {
    try {
        vector<vector<double>> tau;
        vector<vector<double>> tauprime;
        vector<double> Cprime;

        // Đọc tau.csv
        ifstream file("tau.csv");
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
        ifstream file2("tauprime.csv");
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

        // Đọc Cprime.csv
        ifstream file3("Cprime.csv");
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

        int UAVs = 1; // Số lượng UAV
        int n = tau.size(); // n = c+2

        IloEnv env;
        IloModel model(env);

        IloNumVar Z(env);
        IloExpr z(env);
        z = Z;
        model.add(IloMinimize(env, z));

        // Biến quyết định x[i][j]
        IloArray<IloBoolVarArray> x(env, n-1 );
        for (int i = 0; i < n - 1; i++) {
            x[i] = IloBoolVarArray(env, n);
        }
        
        // Biến quyết định y[i][v]
        IloArray<IloBoolVarArray> y(env, n);
        for (int i = 0; i < n; i++) {
            y[i] = IloBoolVarArray(env, UAVs);
        }
        // Ràng buộc s1
        IloExpr st1(env);
        for (int i = 0; i < n - 1; i++) {
            for (int j = 1; j < n; j++) {
                if (j != i)
                    st1 += tau[i][j] * x[i][j];
            }
        }
        model.add(Z >= st1);
        st1.end();

        // Ràng buộc s2
        IloExpr st2(env);
        for (int v = 0; v < UAVs; v++) {
            for (int i = 0; i < n; i++) {
                for (int c : Cprime) {
                    if (i == c) {
                        st2 += (tauprime[0][i] + tauprime[i][n - 1]) * y[i][v];
                        break;
                    }
                }
            }
        }
        model.add(Z >= st2);
        st2.end();

        // Ràng buộc s3
        for (int j = 1; j < n - 1; j++) {
            IloExpr st3(env);
            for (int i = 0; i < n - 1; i++) {
                if (i != j) {
                    st3 += x[i][j];
                }
            }
            for (int v = 0; v < UAVs; v++) {
                for (int c : Cprime) {
                    if (j == c) {
                        st3 += y[j][v];
                        break;
                    }
                }
            }
            model.add(st3 == 1);
        }

        // Ràng buộc s4
        IloExpr st4(env);
        for (int i = 1; i < n; i++) {
            st4 += x[0][i];
        }
        model.add(st4 == 1);
        st4.end();

        // Ràng buộc s5
        IloExpr st5(env);
        for (int i = 0; i < n - 1; i++) {
            st5 += x[i][n - 1];
        }
        model.add(st5 == 1);
        st5.end();

        // Ràng buộc s6
        for (int j = 1; j < n - 1; j++) {
            IloExpr st6(env);
            IloExpr st7(env);
            for (int i = 0; i < n - 1; i++) {
                if (i != j) {
                    st6 += x[i][j];
                }
            }
            for (int k = 1; k < n; k++) {
                if (k != j) {
                    st7 += x[j][k];
                }
            }
            model.add(st6 == st7);
        }

        // Ràng buộc s7
        IloNumVarArray u(env, n, 1, n , ILOINT);
        for (int i = 1; i < n - 1; i++) {
            for (int j = 1; j < n; j++) {
                if (j != i) {
                    model.add(u[i] - u[j] + 1 <= n * (1 - x[i][j]));
                }
            }
        }

        IloCplex cplex(env);
        cplex.extract(model);
        if (!cplex.solve()) {
            throw runtime_error("Failed to solve the model.");
        }
        cout << cplex.getObjValue();

        cplex.clear();
        env.end();
    }
    catch (const IloException& e) {
        cerr << "CPLEX Error: " << e.getMessage() << endl;
    }
    catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
    }
    catch (...) {
        cerr << "An unexpected error occurred." << endl;
    }

    return 0;
}
