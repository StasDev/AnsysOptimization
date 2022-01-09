//
//  Optimization.cpp
//  BladeOptimization
//
//  Created by MyMac on 03.01.2022.
//

#include "Optimization.hpp"

My_Random_Int_f::My_Random_Int_f(int minValue, int maxValue):  dist{minValue, maxValue} {}
int My_Random_Int_f::operator()() {
    return dist(re);
}

//Initialization static members
//Skin
unsigned DESIGN_VARIABLES::skinBitsPerLayer = BITS_PER_SKIN_LAYER;
int DESIGN_VARIABLES::skinMinAngle = SKIN_MIN_ANGLE;
int DESIGN_VARIABLES::skinMaxAngle = SKIN_MAX_ANGLE;
//Tuning mass
unsigned DESIGN_VARIABLES::tmBitsPerR = BITS_PER_TM_R;
int DESIGN_VARIABLES::tmMinR = TM_MIN_R;
int DESIGN_VARIABLES::tmMaxR = TM_MAX_R;
unsigned DESIGN_VARIABLES::tmBitsPerL = BITS_PER_TM_L;
int DESIGN_VARIABLES::tmMinL = TM_MIN_L;
int DESIGN_VARIABLES::tmMaxL = TM_MAX_L;
unsigned DESIGN_VARIABLES::tmBitsPerX = BITS_PER_TM_X;
float DESIGN_VARIABLES::tmMinX = TM_MIN_X;
float DESIGN_VARIABLES::tmMaxX = TM_MAX_X;
unsigned DESIGN_VARIABLES::tmBitsPerY = BITS_PER_TM_Y;
float DESIGN_VARIABLES::tmMinY = TM_MIN_Y;
float DESIGN_VARIABLES::tmMaxY = TM_MAX_Y;
//Spar
unsigned DESIGN_VARIABLES::sparBitsPerLayer = BITS_PER_SPAR_LAYER;
int DESIGN_VARIABLES::sparMinAngle = SPAR_MIN_ANGLE;
int DESIGN_VARIABLES::sparMaxAngle = SPAR_MAX_ANGLE;
unsigned DESIGN_VARIABLES::sparBitsPerWallPosition = BITS_PER_SPAR_WALL_POSITION;
int DESIGN_VARIABLES::sparMinWallPosition = SPAR_WALL_MIN_POSITION;
int DESIGN_VARIABLES::sparMaxWallPosition = SPAR_WALL_MAX_POSITION;
unsigned DESIGN_VARIABLES::sparBitsPerWallAngle = BITS_PER_SPAR_WALL_ANGLE;
int DESIGN_VARIABLES::sparMinWallAngle = SPAR_WALL_MIN_ANGLE;
int DESIGN_VARIABLES::sparMaxWallAngle = SPAR_WALL_MAX_ANGLE;

std::vector<int> DESIGN_VARIABLES::arrSkinSparAngles(pow(2, BITS_PER_SKIN_LAYER));
std::vector<int> DESIGN_VARIABLES::arrTMR(pow(2, BITS_PER_TM_R));
std::vector<int> DESIGN_VARIABLES::arrTML(pow(2, BITS_PER_TM_L));
std::vector<float> DESIGN_VARIABLES::arrTMX(pow(2, BITS_PER_TM_X));
std::vector<float> DESIGN_VARIABLES::arrTMY(pow(2, BITS_PER_TM_Y));
std::vector<int> DESIGN_VARIABLES::arrSparBackWallAngles(pow(2, BITS_PER_SPAR_WALL_ANGLE));
std::vector<int> DESIGN_VARIABLES::arrSparBackWallPositions(pow(2, BITS_PER_SPAR_WALL_POSITION));

DESIGN_VARIABLES::DESIGN_VARIABLES(unsigned skinNumberOfLayrers, unsigned sparNumberOfLayrers): doesBladeHasSpar(sparNumberOfLayrers), skinNumberOfLayrers{skinNumberOfLayrers}, sparNumberOfLayrers{sparNumberOfLayrers}, sizeInBits{skinBitsPerLayer * skinNumberOfLayrers + sparBitsPerLayer * sparNumberOfLayrers + }, binaryEncoding(sizeInBits) {
    My_Random_Int_f RandNumber01(0, 1);
    for (int j = 0; j < sizeInBits; j++)
        binaryEncoding[j] = RandNumber01();
}
DESIGN_VARIABLES::DESIGN_VARIABLES(const DESIGN_VARIABLES &arr): numberSkinLayers{arr.numberSkinLayers}, bitsPerLayer{arr.bitsPerLayer}, sizeInBits{arr.sizeInBits}, minAngle{arr.minAngle}, maxAngle{arr.maxAngle}, stepAngles{arr.stepAngles}, binaryEncoding(arr.binaryEncoding), cost{arr.cost} {}
DESIGN_VARIABLES &DESIGN_VARIABLES::operator=(const DESIGN_VARIABLES &arr) {
    numberSkinLayers = arr.numberSkinLayers;
    bitsPerLayer = arr.bitsPerLayer;
    sizeInBits = arr.sizeInBits;
    minAngle = arr.minAngle;
    maxAngle = arr.maxAngle;
    stepAngles = arr.stepAngles;
    cost = arr.cost;
    for (int i = 0; i < sizeInBits; i++)
        binaryEncoding[i] = arr.binaryEncoding[i];
    return *this;
}
unsigned DESIGN_VARIABLES::GetNumberOfBits() const {
    return sizeInBits;
}
float DESIGN_VARIABLES::GetAngle(unsigned layer) const {
    assert(layer <= numberSkinLayers);
    unsigned indexFirstBit = bitsPerLayer * (layer - 1);
    unsigned positionAngle = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + bitsPerLayer; j++)
        positionAngle += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return minAngle + stepAngles * positionAngle;
}
void DESIGN_VARIABLES::SetApproxAngle(float angle, unsigned layer) {
    unsigned positionAngle = (angle - minAngle) / stepAngles;
    unsigned indexFirstBit = bitsPerLayer * (layer - 1);
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + bitsPerLayer; j++) {
        binaryEncoding[j] = positionAngle & oneBit;
        positionAngle >>= 1; // equiv /= 2
    }
}
void DESIGN_VARIABLES::WriteAnglesInFileWithPath(const std::string &path, const std::string &nameWithExtension) const {
    std::ofstream out(path + nameWithExtension, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    for (int layer = 1; layer <= numberSkinLayers; layer++)
        out << GetAngle(layer) << '\n';
    out.close();
}
float DESIGN_VARIABLES::ReadSafetyFactorFromFileWithPath(const std::string &path, const std::string &nameWithExtension) {
    std::ifstream in(path + nameWithExtension);
    float cost = -1;
    while (in >> cost) {}
    return cost;
}
void DESIGN_VARIABLES::SetCost(float c) {
    cost = c;
}
std::ostream &operator<<(std::ostream &os, const DESIGN_VARIABLES &arr) {
    for (int layer = 1; layer <= arr.numberSkinLayers; layer++ ) {
        os << "Layer " << layer << " has angle " << arr.GetAngle(layer) << '\n';
    }
    return os;
}

Cost_Angles_f::Cost_Angles_f(std::list<DESIGN_VARIABLES> &calculatedSkins, float minAcceptableCost, float maxAcceptableCost): calculatedSkins{calculatedSkins}, minAcceptableCost{minAcceptableCost}, maxAcceptableCost{maxAcceptableCost} {}
float Cost_Angles_f::CheckCost(const DESIGN_VARIABLES &skin) const {
    if(calculatedSkins.empty())
        return -1;
    bool isKeyAlreadyCalculated;
    for (auto node = calculatedSkins.begin(); node != calculatedSkins.end(); node++) {
        isKeyAlreadyCalculated = true;
        assert(node->sizeInBits == skin.sizeInBits);
        for (int j = 0; j < skin.sizeInBits; j++) {
            if (node->binaryEncoding[j] != skin.binaryEncoding[j]) {
                isKeyAlreadyCalculated = false;
                break;
            }
        }
        if (isKeyAlreadyCalculated)
            return node->cost;
    }
    return -1;
}
void Cost_Angles_f::ImposePenalty(float &cost, double r1, double r2, double beta) const {
        //    Without death penalty approach
        //    Want (minAcceptableCost < cost < maxAcceptableCost) that equivalent to (minAcceptableCost - cost < 0 && cost - maxAcceptableCost < 0)
    cost = abs((minAcceptableCost + maxAcceptableCost) / 2 - cost) + r1 * pow(std::max(0.f, minAcceptableCost - cost), beta) + r2 * pow(std::max(0.f, cost - maxAcceptableCost), beta);
}

    //Simple cost functions for debugging
float TestCF_SumSquares(int dimension, const DESIGN_VARIABLES &x){
    float cost = 0;
    for (int j = 1; j <= dimension; j++) {
        cost += pow(x.GetAngle(j), 2);
    }
    return cost;
}
float TestCF_Ackley(int dimension, const DESIGN_VARIABLES &x){
    float cost = 0;
    float a = 20;
    float b = 0.2;
    float c = 2 * M_PI;
    float d = dimension;
    float sumX = 0;
    float sumCosX = 0;
    for (int j = 1; j <= d; j++) {
        sumX += pow(x.GetAngle(j), 2);
        sumCosX += cos(c * x.GetAngle(j));
    }
    cost = -a * exp(-b * sqrt(sumX/d)) - exp(sumCosX/d) + a + exp(1);
    return cost;
}


float Cost_Angles_f::operator()(DESIGN_VARIABLES &skin) {
        //    Check was cost already calculated?
    float cost = CheckCost(skin);
    if (cost > 0)
        return cost;
//    *****************************
//    Test on simple cost functions:
//    cost = TestCF_SumSquares(skin.numberSkinLayers, skin);
//    cost = TestCF_Ackley(skin.numberSkinLayers, skin);
//    *****************************
    skin.WriteAnglesInFileWithPath();
/*
     //    Ansys calculates safetyFactor blade from binaryEncoding which was wrote in file Angles.txt
     Py_Initialize();
     FILE* fp = fopen("/Users/stanislavdumanskij/Documents/GraphiteWork_small/Projects/HelicopterMianRotor/BladeOptimizationCpp/BladeOptimizationCpp/PythonFiles/test_script.py", "r");
     PyRun_AnyFile(fp, "/Users/stanislavdumanskij/Documents/GraphiteWork_small/Projects/HelicopterMianRotor/BladeOptimizationCpp/BladeOptimizationCpp/PythonFiles/test_script.py");
     Py_Finalize();
     
     cost = skin.ReadSafetyFactorFromFileWithPath();
     ImposePenalty(cost);
*/
    skin.SetCost(cost);
    calculatedSkins.push_back(skin);
    return cost;
}

SIMPLE_GA::SIMPLE_GA(unsigned sizePopulation, unsigned maxGenerationNumber, float minAcceptableCost, float maxAcceptableCost, float mutationRate): sizePopulation{sizePopulation}, maxGenerationNumber{maxGenerationNumber}, mutationRate{mutationRate}, population(sizePopulation), calculatedSkins{}, Cost{calculatedSkins, minAcceptableCost, maxAcceptableCost} {
    bestIndivid.SetCost(std::numeric_limits<float>::infinity());
    Optimization();
    std::cout << "\n\n Best individual: \n" << bestIndivid << "It has cost is " << bestIndivid.cost << '\n';
    bestIndivid.WriteAnglesInFileWithPath();
    std::cout << "Best individ cost is " << bestIndivid.ReadSafetyFactorFromFileWithPath() << '\n';
}

std::vector<unsigned> SIMPLE_GA::Selection() const {
    unsigned indexParent1;
    unsigned indexParent2;
    float totalCost = 0;
    for (auto &ind : population)
        totalCost += ind.cost;
    std::vector<float> indCost(sizePopulation);
    for (int j = 0; j < sizePopulation; j++)
        indCost[j] = population[j].cost / totalCost * 100;
    
        //    for (int j = 0; j < sizePopulation; j++)
        //        std::cout << indCost[j] << '\n';
    
    My_Random_Int_f RandomNumber1_100(1, 100);
    unsigned r = RandomNumber1_100();
    unsigned wheel = 0;
    unsigned indexIndivid = 0;
    while (indexIndivid < sizePopulation && wheel < r) {
        wheel += indCost[indexIndivid];
        indexIndivid++;
    }
    indexParent1 = --indexIndivid;
    r = RandomNumber1_100();
    wheel = 0;
    indexIndivid = 0;
    while (indexIndivid < sizePopulation && wheel < r) {
        wheel += indCost[indexIndivid];
        indexIndivid++;
    }
    indexParent2 = --indexIndivid;
    return std::vector<unsigned>{indexParent1, indexParent2};
}
void SIMPLE_GA::Crossover(unsigned indexParent1, unsigned indexParent2) {
    DESIGN_VARIABLES children1{population[indexParent1]};
    DESIGN_VARIABLES children2{population[indexParent2]};
    My_Random_Int_f RandomNumber(1, children1.GetNumberOfBits() - 1);
    unsigned crossoverPoint = RandomNumber();
    for (unsigned j = crossoverPoint; j < children1.GetNumberOfBits(); j++) {
        children1.binaryEncoding[j] = population[indexParent2].binaryEncoding[j];
        children2.binaryEncoding[j] = population[indexParent1].binaryEncoding[j];
    }
    children.push_back(children1);
    children.push_back(children2);
}
void SIMPLE_GA::Mutation() {
    My_Random_Int_f RandomNumber1_100(1, 100);
    for (DESIGN_VARIABLES &ind : children) {
        for (unsigned j = 0; j < ind.GetNumberOfBits(); j++) {
            if (RandomNumber1_100() <= 100 * mutationRate)
                ind.binaryEncoding[j] = (ind.binaryEncoding[j] == true ? false : true);
        }
    }
}

DESIGN_VARIABLES SIMPLE_GA::Optimization() {
    unsigned generation = 1;
    for (auto &ind : population)
        Cost(ind);
    std::vector<unsigned> indexParent(2);
    while (generation <= maxGenerationNumber) {
        for (unsigned j = 0; j < sizePopulation / 2; j++) {
            indexParent = Selection();
            Crossover(indexParent[0], indexParent[1]);
        }
        Mutation();
        for (auto &ind : children) {
            Cost(ind);
            if (ind.cost < bestIndivid.cost)
                bestIndivid = ind;
        }
        population = children;
        children.clear();
        generation++;
    }
    return bestIndivid;
}
