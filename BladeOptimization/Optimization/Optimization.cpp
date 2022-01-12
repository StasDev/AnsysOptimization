//
//  Optimization.cpp
//  BladeOptimization
//
//  Created by MyMac on 03.01.2022.
//

#include "Optimization.hpp"

My_Random_Int_f::My_Random_Int_f(int minValue, int maxValue): dist{minValue, maxValue} {}
int My_Random_Int_f::operator()() {
    return dist(re);
}

//Initialization static members
//Skin
unsigned DESIGN_VARIABLES::skinBitsPerLayer = BITS_PER_SKIN_LAYER;
int DESIGN_VARIABLES::skinMinAngle = SKIN_MIN_ANGLE;
int DESIGN_VARIABLES::skinMaxAngle = SKIN_MAX_ANGLE;
//Tuning mass
unsigned DESIGN_VARIABLES::tmBitsPerRadius = BITS_PER_TM_R;
int DESIGN_VARIABLES::tmMinRadius = TM_MIN_R;
int DESIGN_VARIABLES::tmMaxRadius = TM_MAX_R;
unsigned DESIGN_VARIABLES::tmBitsPerLength = BITS_PER_TM_L;
int DESIGN_VARIABLES::tmMinLength = TM_MIN_L;
int DESIGN_VARIABLES::tmMaxLength = TM_MAX_L;
unsigned DESIGN_VARIABLES::tmBitsPerPositionXY = BITS_PER_TM_XY;
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
std::vector<int> DESIGN_VARIABLES::arrTMRadius(pow(2, BITS_PER_TM_R));
std::vector<int> DESIGN_VARIABLES::arrTMLength(pow(2, BITS_PER_TM_L));
std::vector<int> DESIGN_VARIABLES::arrSparWallAngles(pow(2, BITS_PER_SPAR_WALL_ANGLE));
std::vector<int> DESIGN_VARIABLES::arrSparWallPositions(pow(2, BITS_PER_SPAR_WALL_POSITION));

void DESIGN_VARIABLES::FillDirectAdressingTable(std::vector<int> &arr, unsigned numberOfBits, int minValue, int maxValue) {
    float step = (maxValue - minValue) / (pow(2,numberOfBits) - 1);
    for (int k = 0; k < pow(2, numberOfBits); k++)
        arr[k] = minValue + k * step;
}

void DESIGN_VARIABLES::FillAllDirectAdressingTables(bool hasSpar) {
    FillDirectAdressingTable(arrSkinSparAngles, skinBitsPerLayer, skinMinAngle, skinMaxAngle);
    FillDirectAdressingTable(arrTMRadius, tmBitsPerRadius, tmMinRadius, tmMaxRadius);
    FillDirectAdressingTable(arrTMLength, tmBitsPerLength, tmMinLength, tmMaxLength);
    if (hasSpar) {
        FillDirectAdressingTable(arrSparWallAngles, sparBitsPerWallAngle, sparMinWallAngle, sparMaxWallAngle);
        FillDirectAdressingTable(arrSparWallPositions, sparBitsPerWallPosition, sparMinWallPosition, sparMaxWallPosition);
    }
}

DESIGN_VARIABLES::DESIGN_VARIABLES(unsigned skinNumberOfLayers, unsigned sparNumberOfLayers): doesBladeHasSpar(sparNumberOfLayers), skinNumberOfLayers{skinNumberOfLayers}, sparNumberOfLayers{sparNumberOfLayers}, sizeInBits{skinBitsPerLayer * skinNumberOfLayers + tmBitsPerRadius + tmBitsPerLength + tmBitsPerPositionXY}, binaryEncoding(sizeInBits) {
    My_Random_Int_f RandNumber01(0, 1);
    FillAllDirectAdressingTables(doesBladeHasSpar);
    if (doesBladeHasSpar)
        sizeInBits += sparNumberOfLayers * sparBitsPerLayer + sparBitsPerWallAngle + sparBitsPerWallPosition;
    for (int j = 0; j < sizeInBits; j++)
        binaryEncoding[j] = RandNumber01();
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

void DESIGN_VARIABLES::SetTMRadius(int r) {
    assert(r > 0);
    float stepR = (tmMaxRadius - tmMinRadius) / (pow(2,tmBitsPerRadius) - 1);
    unsigned keyR = (r - tmMinRadius) / stepR; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + tmBitsPerRadius; j++) {
        binaryEncoding[j] = keyR & oneBit;
        keyR >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetTMRadius() const {
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer;
    unsigned keyR = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + tmBitsPerRadius; j++)
        keyR += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrTMRadius[keyR];
}
void DESIGN_VARIABLES::SetTMLength(int L) {
    assert(L > 0);
    float stepL = (tmMaxLength - tmMinLength) / (pow(2,tmBitsPerLength) - 1);
    unsigned keyL = (L - tmMinLength) / stepL; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerRadius;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + tmBitsPerLength; j++) {
        binaryEncoding[j] = keyL & oneBit;
        keyL >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetTMLength() const {
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerRadius;
    unsigned keyL = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + tmBitsPerLength; j++)
        keyL += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrTMLength[keyL];
}
void DESIGN_VARIABLES::SetTMPosition(unsigned keyPosition) {
    assert(keyPosition < pow(2, tmBitsPerPositionXY));
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerRadius + tmBitsPerLength;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + tmBitsPerPositionXY; j++) {
        binaryEncoding[j] = keyPosition & oneBit;
        keyPosition >>= 1; // equiv /= 2
    }
}
unsigned DESIGN_VARIABLES::GetTMPosition() const {
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerRadius + tmBitsPerLength;
    unsigned keyPosition = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + tmBitsPerPositionXY; j++)
        keyPosition += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return keyPosition;
}

void DESIGN_VARIABLES::SetApproxSparAngle(unsigned layer, int angle) {
    assert(doesBladeHasSpar && (layer <= sparNumberOfLayers) && (sparMinAngle <= angle) && (angle <= sparMaxAngle));
    float stepByAngle = (sparMaxAngle - sparMinAngle) / (pow(2,sparBitsPerLayer) - 1);
    unsigned keyAngle = (angle - sparMinAngle) / stepByAngle; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerRadius + tmBitsPerLength + tmBitsPerPositionXY + sparBitsPerLayer * (layer - 1);
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + sparBitsPerLayer; j++) {
        binaryEncoding[j] = keyAngle & oneBit;
        keyAngle >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetSparAngle(unsigned layer) const {
    assert(doesBladeHasSpar && layer <= sparNumberOfLayers);
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerRadius + tmBitsPerLength + tmBitsPerPositionXY + sparBitsPerLayer * (layer - 1);
    unsigned keyAngle = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + sparBitsPerLayer; j++)
        keyAngle += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrSkinSparAngles[keyAngle];
}
void DESIGN_VARIABLES::SetSparWallAngle(int wallAngle) {
    assert(doesBladeHasSpar);
    float stepWA = (sparMaxWallAngle - sparMinWallAngle) / (pow(2,sparBitsPerWallAngle) - 1);
    unsigned keyWA = (wallAngle - sparMinWallAngle) / stepWA; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerRadius + tmBitsPerLength + tmBitsPerPositionXY + sparNumberOfLayers * sparBitsPerLayer;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + sparBitsPerWallAngle; j++) {
        binaryEncoding[j] = keyWA & oneBit;
        keyWA >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetSparWallAngle() const{
    assert(doesBladeHasSpar);
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerRadius + tmBitsPerLength + tmBitsPerPositionXY + sparNumberOfLayers * sparBitsPerLayer;
    unsigned keyWA = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + sparBitsPerWallAngle; j++)
        keyWA += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrSparWallAngles[keyWA];
}
void DESIGN_VARIABLES::SetSparWallPosition(int xD) {
    assert(doesBladeHasSpar);
    float stepWP = (sparMaxWallPosition - sparMinWallPosition) / (pow(2,sparBitsPerWallPosition) - 1);
    unsigned keyWP = (xD - sparMinWallPosition) / stepWP; // approximation
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerRadius + tmBitsPerLength + tmBitsPerPositionXY + sparNumberOfLayers * sparBitsPerLayer + sparBitsPerWallAngle;
    unsigned oneBit = 1;
    for (int j = indexFirstBit; j < indexFirstBit + sparBitsPerWallPosition; j++) {
        binaryEncoding[j] = keyWP & oneBit;
        keyWP >>= 1; // equiv /= 2
    }
}
int DESIGN_VARIABLES::GetSparWallPosition() const{
    assert(doesBladeHasSpar);
    unsigned indexFirstBit = skinNumberOfLayers * skinBitsPerLayer + tmBitsPerRadius + tmBitsPerLength + tmBitsPerPositionXY + sparNumberOfLayers * sparBitsPerLayer + sparBitsPerWallAngle;
    unsigned keyWP = 0;
    for (unsigned j = indexFirstBit; j < indexFirstBit + sparBitsPerWallPosition; j++)
        keyWP += binaryEncoding[j] * pow(2, j - indexFirstBit);
    return arrSparWallAngles[keyWP];
}

void DESIGN_VARIABLES::SetCost(float c) {
    cost = c;
}
float DESIGN_VARIABLES::GetCost() const {
    return cost;
}

void DESIGN_VARIABLES::WriteInFileWithPath(const std::string &path, const std::string &nameWithExtension) const {
   //Write object data in file
}
//float ReadSafetyFactorFromFileWithPath(const std::string &path, const std::string &nameWithExtension) {
//}
std::ostream &operator<<(std::ostream &os, const DESIGN_VARIABLES &x) {
    for (int layer = 1; layer <= x.skinNumberOfLayers; layer++ ) {
        os << "Layer " << layer << " has angle " << x.GetSkinAngle(layer) << '\n';
//        Other variables ...
    }
    return os;
}

//Weights for cost functions
float Cost_f::weightMass = WEIGHT_MASS;
float Cost_f::weightNatrualFrequencies = WEIGHT_NATURAL_FREQUENCIES;
float Cost_f::weightStrength = WEIGHT_STRENGTH;
//Weights for penalty (constraints) functions
float Cost_f::weightSigma = WEIGHT_SIGMA;
float Cost_f::weightUZ = WEIGHT_UZ;
float Cost_f::weightUR = WEIGHT_UR;
float Cost_f::weightUTwist = WEIGHT_UTWIST;
// List of blades with already calculated costs
std::list<DESIGN_VARIABLES> Cost_f::calculatedBlades;

float Cost_f::OmegaMin = MAIN_ROTOR_MIN_SPEED;
float Cost_f::OmegaMax = MAIN_ROTOR_MAX_SPEED;
float Cost_f::rNF = R_NF;
float Cost_f::betaNF = BETA_NF;
float Cost_f::safetyFactorMin = SAFETY_FACTOR_MIN;
float Cost_f::safetyFactorMax = SAFETY_FACTOR_MAX;
float Cost_f::rSF = R_SF;
float Cost_f::betaSF = BETA_SF;
float Cost_f::maxAcceptableBiasTipFlap = MAX_ACCEPTABLE_BIAS_TIP_FLAP;
float Cost_f::rBTF = R_BTF;
float Cost_f::betaBTF = BETA_BTF;
float Cost_f::maxAcceptableTipSectionTwist = MAX_ACCEPTABLE_TWIST_ANGLE;
float Cost_f::rTwist = R_TWIST;
float Cost_f::betaTwist = BETA_TWIST;

float Cost_f::CheckCost(const DESIGN_VARIABLES &blade) const {
    if(calculatedBlades.empty())
        return -1;
    bool isBladeAlreadyConsidered;
    for (auto node = calculatedBlades.begin(); node != calculatedBlades.end(); node++) {
        isBladeAlreadyConsidered = true;
        assert(node->GetSizeInBits() == blade.GetSizeInBits());
        for (int j = 0; j < blade.GetSizeInBits(); j++) {
            if (node->binaryEncoding[j] != blade.binaryEncoding[j]) {
                isBladeAlreadyConsidered = false;
                break;
            }
        }
        if (isBladeAlreadyConsidered)
            return node->cost;
    }
    return -1;
}

float Cost_f::CostMass(const DESIGN_VARIABLES &blade) {
    float massBlade; // <= ANSYS
    return massBlade / massBaselineBlade;
}
float Cost_f::CostNaturalFrequencies(const DESIGN_VARIABLES &blade, unsigned quantityConsideredNatrualFrequencies, unsigned quantityConsideredAirloadHarmonics) {
//    From modal analysis in ANSYS we have two arrarys nFLow[] and nFHigh[] of monotonically increasing natural frequensies of the considered blade for main rotor rotate frequencies OmegaMin and OmegaMax, resprctively, where [OmegaMin, OmegaMax] - interval of work frequencies of the main rotor
    auto nFMin = std::make_unique<float[]>(quantityConsideredNatrualFrequencies); // <= ANSYS
    auto nFMax = std::make_unique<float[]>(quantityConsideredNatrualFrequencies); // <= ANSYS
//    If displacement arrangement all natrual friquencies for low and high main rotor speeds the same then we can use next simple way to approximation fro frequencies curves
//    omega_i = a_i Omega^2 + b_i, Omega in [OmegaMin, OmegaMax], omega_j -- j-th natrual frequency of the blande, a_j and b_j - coefficients which can be finded condiditons omega_j(OmegaMin) = nFMin[j] and omega_j(OmegaMax) = nFMax[j]
    float minDist = std::numeric_limits<float>::infinity();
    auto dist = minDist;
    float a; // equiv a_i
    float b; // equiv b_i
    float OmegaRes1;
    float OmegaRes2;
    float D;
    for (int j = 0; j < quantityConsideredNatrualFrequencies; j++) {
        a = (nFMax[j] - nFMin[j]) / (pow(OmegaMax, 2) - pow(OmegaMin, 2));
        b = nFMin[j] - a * pow(OmegaMin, 2);
        for (int k = 0; k < quantityConsideredAirloadHarmonics; k++) {
            D = pow(k, 2) - 4 * a * b;
            OmegaRes1 = (k - sqrt(D)) / (2 * a);
            OmegaRes2 = (k + sqrt(D)) / (2 * a);
            dist = std::min(abs((OmegaMin + OmegaMax) / 2 - OmegaRes1), abs(OmegaRes2 - (OmegaMin + OmegaMax) / 2));
            if (dist < minDist)
                minDist = dist;
        }
    }
    float cost = - minDist;
    float penalty = rNF * pow(std::max(0.f, (OmegaMax - OmegaMin) / 2 + OmegaMax / 15 - minDist), betaNF);
    return cost + penalty;
}
float Cost_f::CostStress(const DESIGN_VARIABLES &blade) {
    float maxSigmaEq; // <= ANSYS
    return maxSigmaEq / maxEqStressInBaselineBlade;
}

float Cost_f::CostPenalty(const DESIGN_VARIABLES &blade) {
//    Strength constraints - maximum equivalent stresses
    float safetyFactor; // <= ANSYS
    float g_strength = rSF * pow(std::max(0.f, abs((safetyFactorMin + safetyFactorMax) / 2 - safetyFactor) - (safetyFactorMax + safetyFactorMin) / 2), betaSF);
//    Distortion of the blade shape = bias tip of the blade + twisting/detwisting
    float biasTipFlap; // <= ANSYS
    float g_uZ = rBTF * pow(std::max(0.f, abs(biasTipFlap) - maxAcceptableBiasTipFlap), betaBTF);
//    float biasTipRadius; // <= ANSYS
//    float g_uR = ;//biasTipRadius;
    float twistTipSection; // <= ANSYS
    float g_uTwist = rTwist * pow(std::max(0.f, twistTipSection - maxAcceptableTipSectionTwist), betaTwist);
    
    assert(abs(weightStrength + weightUZ + weightUTwist - 1) < 1e-4); // + weightUR
    return weightStrength * g_strength + weightUZ * g_uZ + weightUTwist * g_uTwist; // + weightUR * g_uR 
}

Cost_f::Cost_f(const DESIGN_VARIABLES &baselineBlade) {
    massBaselineBlade; // <= ANSYS
    maxEqStressInBaselineBlade; // <= ANSYS
}

//Simple cost functions for debugging
float TestCF_SumSquares(int dimension, const DESIGN_VARIABLES &x){
    float cost = 0;
    for (int j = 1; j <= dimension; j++) {
        cost += pow(x.GetSkinAngle(j), 2);
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
        sumX += pow(x.GetSkinAngle(j), 2);
        sumCosX += cos(c * x.GetSkinAngle(j));
    }
    cost = -a * exp(-b * sqrt(sumX/d)) - exp(sumCosX/d) + a + exp(1);
    return cost;
}

float Cost_f::operator()(DESIGN_VARIABLES &blade) {
//    Check was cost already calculated?
    float cost = CheckCost(blade);
    if (cost > 0)
        return cost;
//    *****************************
//    Test on simple cost functions:
//    cost = TestCF_SumSquares(skin.numberSkinLayers, skin);
//    cost = TestCF_Ackley(skin.numberSkinLayers, skin);
//    *****************************
    blade.WriteInFileWithPath();
    cost = weightMass * CostMass(blade) + weightNatrualFrequencies * CostNaturalFrequencies(blade) + weightStrength * CostStress(blade) + CostPenalty(blade);
    blade.SetCost(cost);
    calculatedBlades.push_back(blade);
    return cost;
}

SIMPLE_GA::SIMPLE_GA(const DESIGN_VARIABLES baselineBlade, unsigned populationSize, unsigned maxGenerationNumber, float mutationRate): populationSize{populationSize}, maxGenerationNumber{maxGenerationNumber}, mutationRate{mutationRate}, parents(populationSize), children{}, Cost{baselineBlade} {
    bestIndivid.SetCost(std::numeric_limits<float>::infinity());
    Optimization();
    std::cout << "\n\n Best individual has the next parametrization: \n" << bestIndivid << "\n It has cost is " << bestIndivid.GetCost() << '\n';
}

std::vector<unsigned> SIMPLE_GA::Selection() const {
    float totalParentsCost = 0;
    float maxParentCost = -std::numeric_limits<float>::infinity();
    for (auto &ind : parents) {
        totalParentsCost += ind.GetCost();
        if (maxParentCost < ind.GetCost())
            maxParentCost = ind.GetCost();
    }
    float t = 1.1;
    float fMax = t * maxParentCost;
    float d = populationSize * fMax - totalParentsCost;
    std::vector<float> pSelection(populationSize);
    for (int i = 0; i < populationSize; i++)
        pSelection[i] = (fMax - parents[i].GetCost()) / d;
    std::vector<unsigned> indicesParents(2);
    My_Random_Int_f RandomNumber1_100(1, 100);
    unsigned sector; // equiv individ index
    float r;
    float wheelSectorMax;
    for (int i = 0; i < 2; i++) {
        sector = 0;
        r = RandomNumber1_100() / 100.f;
        wheelSectorMax = pSelection[0];
        while (sector != populationSize && r > wheelSectorMax) {
            wheelSectorMax += pSelection[++sector];
        }
        indicesParents[i] = sector;
    }
    return indicesParents;
}
void SIMPLE_GA::Crossover(const std::vector<unsigned> &indicesParents) {
    std::vector<DESIGN_VARIABLES> ch = {parents[indicesParents[0]], parents[indicesParents[1]]};
    My_Random_Int_f RandomNumber(0, ch[0].GetSizeInBits());
    std::vector<int> crossoverPoints = {RandomNumber(), RandomNumber()};
    if (crossoverPoints[0] > crossoverPoints[1])
        std::swap(crossoverPoints[0], crossoverPoints[1]);
    for (int i = crossoverPoints[0]; i <= crossoverPoints[1]; i++) {
        ch[0].binaryEncoding[i] = parents[indicesParents[1]].binaryEncoding[i];
        ch[1].binaryEncoding[i] = parents[indicesParents[0]].binaryEncoding[i];
    }
    children.push_back(ch[0]);
    children.push_back(ch[1]);
}
void SIMPLE_GA::Mutation() {
    My_Random_Int_f RandomNumber1_100(1, 100);
    for (DESIGN_VARIABLES &ind : children) {
        for (unsigned j = 0; j < ind.GetSizeInBits(); j++) {
            if (RandomNumber1_100() <= 100 * mutationRate)
                ind.binaryEncoding[j] = (ind.binaryEncoding[j] == true ? false : true);
        }
    }
}

DESIGN_VARIABLES SIMPLE_GA::Optimization() {
    unsigned generationNumber = 1;
    for (auto &ind : parents) {
        Cost(ind); // calculate costs all parents and save it in Cost_f list<DESIGN_VARIABLES>
        if (ind.cost < bestIndivid.cost)
            bestIndivid = ind;
    }
    std::vector<unsigned> indicesParent(2);
    while (generationNumber <= maxGenerationNumber) {
//        Create children population
        for (unsigned j = 0; j < populationSize / 2; j++)
            Crossover(Selection());
        Mutation();
//        Esitmate they costs
        for (auto &ind : children) {
            Cost(ind);
            if (ind.cost < bestIndivid.cost)
                bestIndivid = ind;
        }
//        Next generation
        parents = children;
        children.clear();
        generationNumber++;
    }
    return bestIndivid;
}
