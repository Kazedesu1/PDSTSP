#include "Instances.h"
#include <stdexcept>
using namespace std;

bool INSTANCE::loadFromFile(const string& filename)
{
	ifstream file(filename);
	if (!file) {
		cerr << "Không thể mở file!" << filename << endl;
		return 1;
	}

	string line;
	double truckSpeed, droneSpeed;

	// Đọc thông tin đầu file
	getline(file, line); sscanf_s(line.c_str(), "NUM DRONES,%d", &UAVs);
	getline(file, line); sscanf_s(line.c_str(), "TRUCK SPEED,%lf", &truckSpeed);
	getline(file, line); sscanf_s(line.c_str(), "DRONE SPEED,%lf", &droneSpeed);

	// Đọc danh sách tọa độ điểm
	while (std::getline(file, line)) {
		std::istringstream iss(line);
		double id, x, y, type;
		if (iss >> id >> x >> y >> type) {
			nodes.push_back({ id, x, y, type });
		}
	}

	// Tính ma trận khoảng cách
	n = nodes.size();
	tau.resize(n, vector<double>(n));
	tauprime.resize(n, vector<double>(n));

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			tau[i][j] = (abs(nodes[i][1] - nodes[j][1]) + abs(nodes[i][2] - nodes[j][2])) / truckSpeed;
			tauprime[i][j] = sqrt(pow(nodes[i][1] - nodes[j][1], 2) + pow(nodes[i][2] - nodes[j][2], 2)) / droneSpeed;
		}
	}

	//lọc ra Cprime
	for (int i = 0; i < n; i++) {
		if (nodes[i][0] != 0 && nodes[i][3] == 0) {
			Cprime.push_back(i);
		}
		if (nodes[i][0] != 0 && nodes[i][3] == 1) {
			truckonly.push_back(i);
		}

		if (nodes[i][0] != 0)
		{
			C.push_back(i);
		}
	}
	n = nodes.size();
	file.close();
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
