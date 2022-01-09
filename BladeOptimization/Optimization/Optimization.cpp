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
std::vector<int> DESIGN_VARIABLES::arrSparWallAngles(pow(2, BITS_PER_SPAR_WALL_ANGLE));
std::vector<int> DESIGN_VARIABLES::arrSparWallPositions(pow(2, BITS_PER_SPAR_WALL_POSITION));

void DESIGN_VARIABLES::FillAllDirectAdressingTables(bool hasSpar) {
    FillDirectAdressingTable(arrSkinSparAngles, skinBitsPerLayer, skinMinAngle, skinMaxAngle);
    FillDirectAdressingTable(arrTMR, tmBitsPerR, tmMinR, tmMaxR);
    FillDirectAdressingTable(arrTML, tmBitsPerL, tmMinL, tmMaxL);
    FillDirectAdressingTable(arrTMX, tmBitsPerX, tmMinX, tmMaxX);
    FillDirectAdressingTable(arrTMY, tmBitsPerY, tmMinY, tmMaxY);
    if (hasSpar) {
        FillDirectAdressingTable(arrSparWallAngles, sparBitsPerWallAngle, sparMinWallAngle, sparMaxWallAngle);
        FillDirectAdressingTable(arrSparWallPositions, sparBitsPerWallPosition, sparMinWallPosition, sparMaxWallPosition);
    }
}

DESIGN_VARIABLES::DESIGN_VARIABLES(unsigned skinNumberOfLayers, unsigned sparNumberOfLayers): doesBladeHasSpar(sparNumberOfLayers), skinNumberOfLayers{skinNumberOfLayers}, sparNumberOfLayers{sparNumberOfLayers}, sizeInBits{skinBitsPerLayer * skinNumberOfLayers + tmBitsPerR + tmBitsPerL + tmBitsPerX + tmBitsPerY}, binaryEncoding(sizeInBits) {
    My_Random_Int_f RandNumber01(0, 1);
    if (doesBladeHasSpar)
        sizeInBits += sparNumberOfLayers * sparBitsPerLayer + sparBitsPerWallAngle + sparBitsPerWallPosition;
    for (int j = 0; j < sizeInBits; j++)
        binaryEncoding[j] = RandNumber01();
    FillAllDirectAdressingTables(doesBladeHasSpar);
}
DESIGN_VARIABLES::DESIGN_VARIABLES(const DESIGN_VARIABLES &x): doesBladeHasSpar{x.doesBladeHasSpar}, skinNumberOfLayers{x.skinNumberOfLayers}, sparNumberOfLayers{x.sparNumberOfLayers}, sizeInBits{x.sizeInBits}, binaryEncoding(x.binaryEncoding), cost{x.cost} {}
DESIGN_VARIABLES &DESIGN_VARIABLES::operator=(const DESIGN_VARIABLES &x) {
    doesBladeHasSpar = x.doesBladeHasSpar;
    skinNumberOfLayers = x.skinNumberOfLayers;
    sparNumberOfLayers = x.sparNumberOfLayers;
    sizeInBits = x.sizeInBits;
    binaryEncoding = x.binaryEncoding;
    cost = x.cost;
    for (int i = 0; i < sizeInBits; i++)
        binaryEncoding[i] = x.binaryEncoding[i];
    return *this;
}
unsigned DESIGN_VARIABLES::GetSizeInBits() const {
    return sizeInBits;
}

void DESIGN_VARIABLES::SetApproxSkinAngle(unsigned layer, int angle) {
    assert((layer <= skinNumberOfLayers) && (skinMinAngle <= angle) && (angle <= skinMaxAngle));
    float stepByAngle = (skinMaxAngle - skinMinAngle) / (pow(2,skinBitsPerLayer) - 1);
    unsigned keyAngle = (angle - skinMinAngle) / stepByAngle; // approximation
    unsigned indexFirstBit = skinBitsPerLayer * (layer - 1);
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + skinBitsPerLayer; j++) {
        binaryEncoding[j] = keyAngle & oneBit;
        keyAngle >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetSkinAngle(unsigned layer) const {
    assert(layer <= skinNumberOfLayers);
    unsigned indexFirstBit = skinBitsPerLayer * (layer - 1);
    unsigned keyAngle = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + skinBitsPerLayer; j++)
        keyAngle += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrSkinSparAngles[keyAngle];
}

void DESIGN_VARIABLES::SetTMR(int r) {
    assert(r > 0);
    float stepR = (tmMaxR - tmMinR) / (pow(2,tmBitsPerR) - 1);
    unsigned keyR = (r - tmMinR) / stepR; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + tmBitsPerR; j++) {
        binaryEncoding[j] = keyR & oneBit;
        keyR >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetTMR() const {
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer;
    unsigned keyR = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + tmBitsPerR; j++)
        keyR += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrTMR[keyR];
}

void DESIGN_VARIABLES::SetTML(int L) {
    assert(L > 0);
    float stepL = (tmMaxL - tmMinL) / (pow(2,tmBitsPerL) - 1);
    unsigned keyL = (L - tmMinL) / stepL; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + tmBitsPerL; j++) {
        binaryEncoding[j] = keyL & oneBit;
        keyL >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetTML() const {
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR;
    unsigned keyL = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + tmBitsPerL; j++)
        keyL += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrTML[keyL];
}

void DESIGN_VARIABLES::SetTMX(float x) {
    float stepX = (tmMaxX - tmMinX) / (pow(2,tmBitsPerX) - 1);
    unsigned keyX = (x - tmMinX) / stepX; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR + tmBitsPerL;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + tmBitsPerX; j++) {
        binaryEncoding[j] = keyX & oneBit;
        keyX >>= 1; // equiv /= 2
    }
}
float DESIGN_VARIABLES::GetTMX() const {
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR + tmBitsPerL;
    unsigned keyX = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + tmBitsPerX; j++)
        keyX += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrTMX[keyX];
}

void DESIGN_VARIABLES::SetTMY(float y) {
    float stepY = (tmMaxY - tmMinY) / (pow(2,tmBitsPerY) - 1);
    unsigned keyY = (y - tmMinY) / stepY; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR + tmBitsPerL + tmBitsPerX;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + tmBitsPerY; j++) {
        binaryEncoding[j] = keyY & oneBit;
        keyY >>= 1; // equiv /= 2
    }
}
float DESIGN_VARIABLES::GetTMY() const {
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR + tmBitsPerL + tmBitsPerX;
    unsigned keyY = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + tmBitsPerY; j++)
        keyY += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrTMY[keyY];
}

void DESIGN_VARIABLES::SetApproxSparAngle(unsigned layer, int angle) {
    assert(doesBladeHasSpar && (layer <= sparNumberOfLayers) && (sparMinAngle <= angle) && (angle <= sparMaxAngle));
    float stepByAngle = (sparMaxAngle - sparMinAngle) / (pow(2,sparBitsPerLayer) - 1);
    unsigned keyAngle = (angle - sparMinAngle) / stepByAngle; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR + tmBitsPerL + tmBitsPerX + tmBitsPerY + sparBitsPerLayer * (layer - 1);
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + sparBitsPerLayer; j++) {
        binaryEncoding[j] = keyAngle & oneBit;
        keyAngle >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetSparAngle(unsigned layer) const {
    assert(doesBladeHasSpar && layer <= sparNumberOfLayers);
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR + tmBitsPerL + tmBitsPerX + tmBitsPerY + sparBitsPerLayer * (layer - 1);
    unsigned keyAngle = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + sparBitsPerLayer; j++)
        keyAngle += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrSkinSparAngles[keyAngle];
}

void DESIGN_VARIABLES::SetSparWallAngle(int wallAngle) {
    assert(doesBladeHasSpar);
    float stepWA = (sparMaxWallAngle - sparMinWallAngle) / (pow(2,sparBitsPerWallAngle) - 1);
    unsigned keyWA = (wallAngle - sparMinWallAngle) / stepWA; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR + tmBitsPerL + tmBitsPerX + tmBitsPerY + sparNumberOfLayers * sparBitsPerLayer;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + sparBitsPerWallAngle; j++) {
        binaryEncoding[j] = keyWA & oneBit;
        keyWA >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetSparWallAngle() const{
    assert(doesBladeHasSpar);
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR + tmBitsPerL + tmBitsPerX + tmBitsPerY + sparNumberOfLayers * sparBitsPerLayer;
    unsigned keyWA = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + sparBitsPerWallAngle; j++)
        keyWA += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrSparWallAngles[keyWA];
}

void DESIGN_VARIABLES::SetSparWallPosition(int xD) {
    assert(doesBladeHasSpar);
    float stepWP = (sparMaxWallPosition - sparMinWallPosition) / (pow(2,sparBitsPerWallPosition) - 1);
    unsigned keyWP = (xD - sparMinWallPosition) / stepWP; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR + tmBitsPerL + tmBitsPerX + tmBitsPerY + sparNumberOfLayers * sparBitsPerLayer + sparBitsPerWallAngle;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + sparBitsPerWallPosition; j++) {
        binaryEncoding[j] = keyWP & oneBit;
        keyWP >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetSparWallPosition() const{
    assert(doesBladeHasSpar);
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerR + tmBitsPerL + tmBitsPerX + tmBitsPerY + sparNumberOfLayers * sparBitsPerLayer + sparBitsPerWallAngle;
    unsigned keyWP = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + sparBitsPerWallPosition; j++)
        keyWP += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrSparWallAngles[keyWP];
}






void DESIGN_VARIABLES::SetCost(float c) {
    cost = c;
}

std::ostream &operator<<(std::ostream &os, const DESIGN_VARIABLES &x) {
    for (int layer = 1; layer <= x.skinNumberOfLayers; layer++ ) {
        os << "Layer " << layer << " has angle " << arr.GetAngle(layer) << '\n';
//        ...
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
