#ifndef Parameters_hpp
#define Parameters_hpp

constexpr unsigned NUMBER_OF_LAYERS = 2;
constexpr unsigned BITS_PER_LAYER = 6; // Search space has 2^(BITS_PER_LAYER * NUMBER_OF_LAYERS) elements
constexpr float MIN_ANGLE = -89;
constexpr float MAX_ANGLE = 90;
constexpr float MIN_ACCEPTABLE_COST = 1.5;
constexpr float MAX_ACCEPTABLE_COST = 2;
constexpr unsigned SIZE_POPULATION = 20;
constexpr unsigned MAX_GENERATION_NUMBER = 500;
constexpr float PENALTY_R1 = 3;
constexpr float PENALTY_R2 = 2;
constexpr float PENALTY_BETA = 2;
constexpr float MUTATION_RATE = 0.02;

#endif /* Parameters_hpp */
