#pragma once
#ifndef UTILS_H
#define UTILS_H
#include <algorithm>
#include <random>
#include <ctime>
#include <chrono>
#include <vector>
#include <iostream>
#include <math.h>
#include <assert.h>

class Utils
{
public:
    static std::mt19937 mt;
    static int seed;

    static int integer_random_generator(const int& a, const int& b);

    static double real_random_generator(const double& a, const double& b);

    static int biased_selection(const std::vector<double>& vec);

    static void shuffle(std::vector<int>& vec);

    template <typename T>
    static void remove(std::vector<T>& c, T& element) {
        for (auto it = c.begin(); it != c.end(); /* "it" updated inside loop body */) {
            if (*it == element) {
                it = c.erase(it);
                break;
            }
            else {
                ++it;
            }
        }
    }

    template <typename T>
    static void print_vec(std::vector<T>& c) {
        for (auto i : c)
            std::cout << i << " ";
        std::cout << "\n";
    }

    static double round(double val, int n);

    static int sumVec_int(std::vector<int>& vec);

    static int position(std::vector<int>& vec, int element);

    static double polarAngle(double x0, double y0, double x1, double y1, double x2, double y2);

    static double angle(double new_ray, double value);

    static void swap_segment_headtail(std::vector<int>& tour1, std::vector<int>& tour2, int i, int j);
    static void swap_segment_headhead(std::vector<int>& tour1, std::vector<int>& tour2, int i, int j);
private:

};

#endif // UTILS_H
