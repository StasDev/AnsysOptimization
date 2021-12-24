#ifndef Optimization_hpp
#define Optimization_hpp

#include "Libraries.hpp"
#include "Parameters.hpp"
#include <random>
#include </Library/Frameworks/Python.framework/Versions/3.10/include/python3.10/Python.h>

class My_Random_Int_f {
    std::random_device re;
    std::uniform_int_distribution<> dist;
public:
    My_Random_Int_f(int minValue = 0, int maxValue = 90);
    int operator()();
};

class ARRAY_ANGLES {
    unsigned numberOfLayers;
    unsigned bitsPerLayer;
    unsigned sizeInBits;
    float minAngle;
    float maxAngle;
    float stepAngles;
    std::vector<bool> data; // binary representation indexes angles in array [minAngle, minAngle + stepAngles, ..., maxAngle - stepAngles, maxAngle]
    float cost = std::numeric_limits<double>::infinity();
public:
    ARRAY_ANGLES(unsigned numberOfLayers = NUMBER_OF_LAYERS, unsigned bitsPerLayer = BITS_PER_LAYER, float minAngle = -89, float maxAngle = 90);
    ARRAY_ANGLES(const ARRAY_ANGLES &arr);
    ARRAY_ANGLES &operator=(const ARRAY_ANGLES &arr);
    
    unsigned GetNumberOfBits() const;
    float GetAngle(unsigned layer) const;
    void SetApproxAngle(float angle, unsigned layer);
    void WriteAnglesInFileWithPath(const std::string &path = "/Users/stanislavdumanskij/Documents/GraphiteWork_small/Projects/HelicopterMianRotor/BladeOptimizationCpp/", const std::string &nameWithExtension =  "Angles.txt") const;
    float ReadSafetyFactorFromFileWithPath(const std::string &path = "/Users/stanislavdumanskij/Documents/GraphiteWork_small/Projects/HelicopterMianRotor/BladeOptimizationCpp/", const std::string &nameWithExtension =  "Angles.txt");
    void SetCost(float c);
    
    friend std::ostream &operator<<(std::ostream &os, const ARRAY_ANGLES &arr);
    friend class Cost_Angles_f;
    friend class SIMPLE_GA;
};

class Cost_Angles_f {
    float minAcceptableCost;
    float maxAcceptableCost;
    std::list<ARRAY_ANGLES> &calculatedSkins; // reference on list with already calculated variants of blade, in particular costs
    
    float CheckCost(const ARRAY_ANGLES &skin) const; // Check whether skin.cost already calculated if yes then return cost otherwise return -1
    void ImposePenalty(float &cost, double r1 = PENALTY_R1, double r2 = PENALTY_R2, double beta = PENALTY_BETA) const; // new cost function: phi(x) = f(x) + rj * Gj(x), f(x) is cost function without take into account constraints, Gj(x) = [max{0, gj(x)}]^beta, where gj(x) < 0 are constraints and r1, r2, beta are parameters of penalty function.
public:
    Cost_Angles_f(std::list<ARRAY_ANGLES> &calculatedSkins, float minAcceptableCost = MIN_ACCEPTABLE_COST, float maxAcceptableCost = MAX_ACCEPTABLE_COST);
    float operator()(ARRAY_ANGLES &skin);
};

class SIMPLE_GA {
    unsigned sizePopulation;
    unsigned maxGenerationNumber;
    float mutationRate;
    std::vector<ARRAY_ANGLES> population;
    std::vector<ARRAY_ANGLES> children;
    
    std::list<ARRAY_ANGLES> calculatedSkins;
    Cost_Angles_f Cost; // has information about already calculated costs
    ARRAY_ANGLES bestIndivid;
    
    std::vector<unsigned> Selection() const; // select individs from population for cossover, return indexes parents
    void Crossover(unsigned indexParent1, unsigned indexParent2);
    void Mutation();
    
//    const ARRAY_ANGLES &BestIndividInPopulation() const;
public:
    SIMPLE_GA(unsigned sizePopulation = SIZE_POPULATION, unsigned maxGenerationNumber = MAX_GENERATION_NUMBER, float minAcceptableCost = MIN_ACCEPTABLE_COST, float maxAcceptableCost = MAX_ACCEPTABLE_COST, float mutationRate = MUTATION_RATE);
    ARRAY_ANGLES Optimization();
};

#endif /* Optimization_hpp */
