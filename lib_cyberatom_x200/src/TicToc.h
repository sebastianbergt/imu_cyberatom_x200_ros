#ifndef TICTOC_H
#define TICTOC_H

#include <chrono>
#include <iostream>

std::chrono::time_point<std::chrono::system_clock> tstart, tend; 

void tic() {
    tstart = std::chrono::system_clock::now();
}

double toc() {
    tend = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = tend-tstart;
    std::cout << "It took "<< diff.count() <<" s."<< std::endl;
    return diff.count();
}

std::chrono::time_point<std::chrono::system_clock> tstart2, tend2; 

void tic2() {
    tstart2 = std::chrono::system_clock::now();
}

double toc2() {
    tend2 = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = tend2-tstart2;
    std::cout << "It took "<< diff.count() <<" s."<< std::endl;
    return diff.count();
}

#endif //TICTOC_H