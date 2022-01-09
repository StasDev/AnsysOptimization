//
//  Optimization.hpp
//  BladeOptimization
//
//  Created by MyMac on 03.01.2022.
//

#ifndef Optimization_hpp
#define Optimization_hpp

#include "Parameters.hpp"
#include "Libraries.hpp"

class My_Random_Int_f {
    std::random_device re;
    std::uniform_int_distribution<> dist;
public:
    My_Random_Int_f(int minValue = 0, int maxValue = 90);
    int operator()();
};

//Blade consist of skin, tuning mass (tm), foam, and optionally D-spar (spar)
class DESIGN_VARIABLES {
    bool doesBladeHasSpar;
//    Skin
    unsigned skinNumberOfLayrers;
    static unsigned skinBitsPerLayer;
    static int skinMinAngle;
    static int skinMaxAngle;
//    Tuning mass
    static unsigned tmBitsPerR;
    static int tmMinR;
    static int tmMaxR;
    static unsigned tmBitsPerL;
    static int tmMinL;
    static int tmMaxL;
    static unsigned tmBitsPerX;
    static float tmMinX;
    static float tmMaxX;
    static unsigned tmBitsPerY;
    static float tmMinY;
    static float tmMaxY;
//    D-spar
    unsigned sparNumberOfLayrers;
    static unsigned sparBitsPerLayer;
    static int sparMinAngle;
    static int sparMaxAngle;
    static unsigned sparBitsPerWallPosition;
    static int sparMinWallPosition;
    static int sparMaxWallPosition;
    static unsigned sparBitsPerWallAngle;
    static int sparMinWallAngle;
    static int sparMaxWallAngle;
    
//    For decoding of binary representation is used direct addressing scheme;
    static std::vector<int> arrSkinSparAngles;
    static std::vector<int> arrTMR;
    static std::vector<int> arrTML;
    static std::vector<float> arrTMX;
    static std::vector<float> arrTMY;
    static std::vector<int> arrSparBackWallAngles;
    static std::vector<int> arrSparBackWallPositions;
    
    template <typename T>
    static void FillDirectAdressingTable(std::vector<T> &arr, unsigned numberOfBits, T minValue, T maxValue) {
        float step = (maxValue - minValue) / (pow(2,numberOfBits) - 1);
        for (int k = 0; k < pow(2, numberOfBits); k++)
            arr[k] = minValue + k * step;
    }
    
    unsigned sizeInBits;
    std::vector<bool> binaryEncoding; // binary representation indexes angles in array [minAngle, minAngle + stepAngles, ..., maxAngle - stepAngles, maxAngle]
    float cost = std::numeric_limits<double>::infinity();
public:
    DESIGN_VARIABLES(unsigned skinNumberOfLayrers = NUMBER_SKIN_LAYERS, unsigned sparNumberOfLayrers = NUMBER_SPAR_LAYERS);
    DESIGN_VARIABLES(const DESIGN_VARIABLES &arr);
    DESIGN_VARIABLES &operator=(const DESIGN_VARIABLES &arr);
    
    unsigned GetNumberOfBits() const;
    float GetAngle(unsigned layer) const;
    void SetApproxAngle(float angle, unsigned layer);
    void WriteAnglesInFileWithPath(const std::string &path = "/Users/mymac/Documents/Cpp/BladeOptimizationCpp/BladeOptimization/BladeOptimization/Files", const std::string &nameWithExtension =  "Angles.txt") const;
    float ReadSafetyFactorFromFileWithPath(const std::string &path = "/Users/mymac/Documents/Cpp/BladeOptimizationCpp/BladeOptimization/BladeOptimization/Files", const std::string &nameWithExtension =  "Angles.txt");
    void SetCost(float c);
    
    friend std::ostream &operator<<(std::ostream &os, const DESIGN_VARIABLES &arr);
    friend class Cost_Angles_f;
    friend class SIMPLE_GA;
};

class Cost_Angles_f {
    float minAcceptableCost;
    float maxAcceptableCost;
    std::list<DESIGN_VARIABLES> &calculatedSkins; // reference on list with already calculated variants of blade, in particular costs
    
    float CheckCost(const DESIGN_VARIABLES &skin) const; // Check whether skin.cost already calculated if yes then return cost otherwise return -1
    void ImposePenalty(float &cost, double r1 = PENALTY_R1, double r2 = PENALTY_R2, double beta = PENALTY_BETA) const; // new cost function: phi(x) = f(x) + rj * Gj(x), f(x) is cost function without take into account constraints, Gj(x) = [max{0, gj(x)}]^beta, where gj(x) < 0 are constraints and r1, r2, beta are parameters of penalty function.
public:
    Cost_Angles_f(std::list<DESIGN_VARIABLES> &calculatedSkins, float minAcceptableCost = MIN_ACCEPTABLE_COST, float maxAcceptableCost = MAX_ACCEPTABLE_COST);
    float operator()(DESIGN_VARIABLES &skin);
};

class SIMPLE_GA {
    unsigned sizePopulation;
    unsigned maxGenerationNumber;
    float mutationRate;
    std::vector<DESIGN_VARIABLES> population;
    std::vector<DESIGN_VARIABLES> children;
    
    std::list<DESIGN_VARIABLES> calculatedSkins;
    Cost_Angles_f Cost; // has information about already calculated costs
    DESIGN_VARIABLES bestIndivid;
    
    std::vector<unsigned> Selection() const; // select individs from population for cossover, return indexes parents
    void Crossover(unsigned indexParent1, unsigned indexParent2);
    void Mutation();
    
//    const DESIGN_VARIABLES &BestIndividInPopulation() const;
public:
    SIMPLE_GA(unsigned sizePopulation = SIZE_POPULATION, unsigned maxGenerationNumber = MAX_GENERATION_NUMBER, float minAcceptableCost = MIN_ACCEPTABLE_COST, float maxAcceptableCost = MAX_ACCEPTABLE_COST, float mutationRate = MUTATION_RATE);
    DESIGN_VARIABLES Optimization();
};

#endif /* Optimization_hpp */
