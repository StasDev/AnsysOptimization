//
//  main.cpp
//  BladeOptimization
//
//  Created by MyMac on 03.01.2022.
//

#include "Optimization.hpp"

int main(int argc, const char * argv[]) {
//    Create object "test" which constructor does optimization
//    SIMPLE_GA test(SIZE_POPULATION,MAX_GENERATION_NUMBER, MIN_ACCEPTABLE_COST,MAX_ACCEPTABLE_COST, MUTATION_RATE);
    DESIGN_VARIABLES dv;
//    std::cout << dv.GetSizeInBits() << '\n';
    std::vector<bool> b = {true};
    std::cout << sizeof(b) << '\n';
    std::cout << sizeof(dv) << '\n';
    return 0;
}
