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
public: // for debugging
    bool doesBladeHasSpar;
//    Skin
    unsigned skinNumberOfLayers;
    static unsigned skinBitsPerLayer;
    static int skinMinAngle;
    static int skinMaxAngle;
//    Tuning mass
    static unsigned tmBitsPerRadius;
    static int tmMinRadius;
    static int tmMaxRadius;
    static unsigned tmBitsPerLength;
    static int tmMinLength;
    static int tmMaxLength;
    static unsigned tmBitsPerPositionXY;
//    Spar
    unsigned sparNumberOfLayers;
    static unsigned sparBitsPerLayer;
    static int sparMinAngle;
    static int sparMaxAngle;
    static unsigned sparBitsPerWallPosition;
    static int sparMinWallPosition;
    static int sparMaxWallPosition;
    static unsigned sparBitsPerWallAngle;
    static int sparMinWallAngle;
    static int sparMaxWallAngle;
//    Binary representation of individ
    unsigned sizeInBits;
    std::vector<bool> binaryEncoding;
    
    float cost = std::numeric_limits<double>::infinity();
    
//    Arrays needed for decoding of binary representation is used direct addressing scheme
    static std::vector<int> arrSkinSparAngles;
    static std::vector<int> arrTMRadius;
    static std::vector<int> arrTMLength;
    static std::vector<int> arrSparWallAngles;
    static std::vector<int> arrSparWallPositions;
    
    static void FillDirectAdressingTable(std::vector<int> &arr, unsigned numberOfBits, int minValue, int maxValue);
    static void FillAllDirectAdressingTables(bool hasSpar);
    
//public:
    DESIGN_VARIABLES(unsigned skinNumberOfLayers = NUMBER_SKIN_LAYERS, unsigned sparNumberOfLayers = NUMBER_SPAR_LAYERS);
    DESIGN_VARIABLES(const DESIGN_VARIABLES &arr);
    DESIGN_VARIABLES &operator=(const DESIGN_VARIABLES &arr);
    
    unsigned GetSkinNumberOfLayers() const;
    void SetApproxSkinAngle(unsigned layer, int angle);
    int GetSkinAngle(unsigned layer) const;
    
    void SetTMRadius(int r);
    int GetTMRadius() const;
    
    void SetTMLength(int L);
    int GetTMLength() const;
    
    void SetTMPosition(unsigned keyPosition);
    unsigned GetTMPosition() const;
    
    unsigned GetSparNumberOfLayers() const;
    void SetApproxSparAngle(unsigned layer, int angle);
    int GetSparAngle(unsigned layer) const;
    
    void SetSparWallAngle(int angle);
    int GetSparWallAngle() const;
    
    void SetSparWallPosition(int xD);
    int GetSparWallPosition() const;
    
    unsigned GetSizeInBits() const;
    
    void SetCost(float c);
    float GetCost() const;
    
    void WriteInFileWithPath(const std::string &path = "/Users/mymac/Documents/Cpp/BladeOptimizationCpp/BladeOptimization/BladeOptimization/Files", const std::string &nameWithExtension =  "Database.txt") const;
    
    friend std::ostream &operator<<(std::ostream &os, const DESIGN_VARIABLES &x);
    
    friend class Cost_f;
    friend class SIMPLE_GA;
};

class Cost_f {
    static float weightMass;
    static float weightNatrualFrequencies;
    static float weightStrength;
    
    static float weightSigma;
    static float weightUZ;
    static float weightUR;
    static float weightUTwist;
    
    static float OmegaMin;
    static float OmegaMax;
    static float rNF;
    static float betaNF;
    
    static float safetyFactorMin;
    static float safetyFactorMax;
    static float rSF;
    static float betaSF;
    
    static float maxAcceptableBiasTipFlap;
    static float rBTF;
    static float betaBTF;
    
    static float maxAcceptableTipSectionTwist;
    static float rTwist;
    static float betaTwist;
    

    static std::list<DESIGN_VARIABLES> calculatedBlades;
    
    float massBaselineBlade;
    float maxEqStressInBaselineBlade;
    
    float CheckCost(const DESIGN_VARIABLES &skin) const; // Check whether skin.cost already calculated if yes then return cost otherwise return -1
    float CostMass(const DESIGN_VARIABLES &blade); // massBaselineBlade measured in grams
    float CostNaturalFrequencies(const DESIGN_VARIABLES &blade, unsigned quantityConsideredNatrualFrequencies = 10, unsigned quantityConsideredAirloadHarmonics = 10);
    float CostStress(const DESIGN_VARIABLES &blade);
    float CostPenalty(const DESIGN_VARIABLES &blade);
public:
    Cost_f(const DESIGN_VARIABLES baselineBlade = {});
    float operator()(DESIGN_VARIABLES &blade);
};

float TestCF_SumSquares(const DESIGN_VARIABLES &blade);
float TestCF_ley(const DESIGN_VARIABLES &blade);

class SIMPLE_GA {
    unsigned populationSize;
    unsigned maxGenerationNumber;
    float mutationRate;
    std::vector<DESIGN_VARIABLES> parents;
    std::vector<DESIGN_VARIABLES> children;
    DESIGN_VARIABLES bestIndivid;
    
    Cost_f Cost;
    std::vector<unsigned> Selection() const; // method select two individuals from parents population for crossover and returns it indices
    void Crossover(const std::vector<unsigned> &indicesParents);
    void Mutation();
    
//    const DESIGN_VARIABLES &BestIndividInPopulation() const;
public:
    SIMPLE_GA(unsigned populationSize = POPULATION_SIZE, unsigned maxGenerationNumber = MAX_GENERATION_NUMBER, float mutationRate = MUTATION_RATE, const DESIGN_VARIABLES baselineBlade = {});
    DESIGN_VARIABLES Optimization();
};

#endif /* Optimization_hpp */
