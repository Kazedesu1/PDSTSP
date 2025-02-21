#include "utils.h"
//#include <math.h>
#include <cmath>
#include <string>

int Utils::seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();

// dynamic seed
std::mt19937 Utils::mt = std::mt19937(Utils::seed);

// fixed seed
//std::mt19937 Utils::mt = std::mt19937(-855555682);


int Utils::integer_random_generator(const int& a, const int& b) {
    if (b <= a)
        throw std::string("ERROR | int random problem");
    return std::uniform_int_distribution<int>{a, b - 1}(Utils::mt);
}

double Utils::real_random_generator(const double& a, const double& b) {
    if (b <= a)
        throw std::string("ERROR | double random problem");
    return std::uniform_real_distribution<double>{a, b}(Utils::mt);
}

int Utils::biased_selection(const std::vector<double>& vec) {
    std::discrete_distribution<> d(vec.begin(), vec.end());

    return d(mt);
}

void Utils::shuffle(std::vector<int>& vec) {
    std::shuffle(vec.begin(), vec.end(), Utils::mt);
}

double Utils::round(double val, int n)
{
    return std::round(val * std::pow(10, n)) / std::pow(10, n);
}

int Utils::sumVec_int(std::vector<int>& vec)
{
    int sum = 0;
    for (int& i : vec)
        sum += i;
    return sum;
}

int Utils::position(std::vector<int>& vec, int element)
{
    auto it = std::find(vec.begin(), vec.end(), element);
    if (it == vec.end())
    {
        // element not in vector
        assert(false);
        return -1;
    }
    else
    {
        return std::distance(vec.begin(), it);
    }
}

double Utils::polarAngle(double x0, double y0, double x1, double y1, double x2, double y2)
{
    double xvec1 = x1 - x0;
    double yvec1 = y1 - y0;
    double xvec2 = x2 - x0;
    double yvec2 = y2 - y0;
    double a = acos((xvec1 * xvec2 + yvec1 * yvec2) / (sqrt(xvec1 * xvec1 + yvec1 * yvec1) * sqrt(xvec2 * xvec2 + yvec2 * yvec2))) * 180.0 / 3.14159265;

    if (a <= 180.0) {
        return a;
    }
    else {
        return 360.0 - a;
    }
}

double Utils::angle(double new_ray, double value)
{
    double v;
    v = abs(value - new_ray);
    if (v > 180)
        return 360 - v;
    return v;

}

void Utils::swap_segment_headhead(std::vector<int>& tour1, std::vector<int>& tour2, int i, int j)
{
    std::vector<int> part11 = std::vector<int>(tour1.begin(), tour1.begin() + i + 1);
    std::vector<int> part12 = std::vector<int>(tour1.begin() + i + 1, tour1.end());
    std::vector<int> part21 = std::vector<int>(tour2.begin(), tour2.begin() + j + 1); std::reverse(part21.begin(), part21.end());
    std::vector<int> part22 = std::vector<int>(tour2.begin() + j + 1, tour2.end()); std::reverse(part22.begin(), part22.end());

    std::vector<int> concat1;
    std::vector<int> concat2;
    concat1.insert(concat1.end(), part11.begin(), part11.end());
    concat1.insert(concat1.end(), part21.begin(), part21.end());
    concat2.insert(concat2.end(), part22.begin(), part22.end());
    concat2.insert(concat2.end(), part12.begin(), part12.end());

    tour1.clear();
    tour2.clear();
    tour1 = concat1;
    tour2 = concat2;
}

void Utils::swap_segment_headtail(std::vector<int>& tour1, std::vector<int>& tour2, int i, int j)
{
    std::vector<int> part11 = std::vector<int>(tour1.begin(), tour1.begin() + i + 1);
    std::vector<int> part12 = std::vector<int>(tour1.begin() + i + 1, tour1.end());
    std::vector<int> part21 = std::vector<int>(tour2.begin(), tour2.begin() + j + 1);
    std::vector<int> part22 = std::vector<int>(tour2.begin() + j + 1, tour2.end());

    std::vector<int> concat1;
    std::vector<int> concat2;
    concat1.insert(concat1.end(), part11.begin(), part11.end());
    concat1.insert(concat1.end(), part22.begin(), part22.end());
    concat2.insert(concat2.end(), part21.begin(), part21.end());
    concat2.insert(concat2.end(), part12.begin(), part12.end());

    tour1.clear();
    tour2.clear();
    tour1 = concat1;
    tour2 = concat2;
}
