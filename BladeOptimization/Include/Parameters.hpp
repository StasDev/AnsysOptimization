//
//  Parameters.hpp
//  BladeOptimization
//
//  Created by MyMac on 03.01.2022.
//

#ifndef Parameters_hpp
#define Parameters_hpp

constexpr unsigned NUMBER_SKIN_LAYERS = 2;
constexpr unsigned BITS_PER_SKIN_LAYER = 6;
constexpr int SKIN_MIN_ANGLE = -89; // deg
constexpr int SKIN_MAX_ANGLE = 90;  // deg

//Need determine normal values
constexpr unsigned BITS_PER_TM_R = 1;
constexpr int TM_MIN_R = 1; // mm
constexpr int TM_MAX_R = 2; // mm
constexpr unsigned BITS_PER_TM_L = 2;
constexpr int TM_MIN_L = 360; // mm
constexpr int TM_MAX_L = 570; // mm
constexpr unsigned BITS_PER_TM_X = 1;
constexpr float TM_MIN_X = 1; // mm
constexpr float TM_MAX_X = 2; // mm
constexpr unsigned BITS_PER_TM_Y = 2;
constexpr float TM_MIN_Y = -1;
constexpr float TM_MAX_Y = 1;

constexpr unsigned NUMBER_SPAR_LAYERS = 2;
constexpr unsigned BITS_PER_SPAR_LAYER = 6;
constexpr int SPAR_MIN_ANGLE = -89; // deg
constexpr int SPAR_MAX_ANGLE = 90;  // deg

constexpr unsigned BITS_PER_SPAR_WALL_ANGLE = 4;
constexpr int SPAR_WALL_MIN_ANGLE = -30; // deg
constexpr int SPAR_WALL_MAX_ANGLE = 30;  // deg

constexpr unsigned BITS_PER_SPAR_WALL_POSITION = 4;
constexpr int SPAR_WALL_MIN_POSITION = 2;   // mm
constexpr int SPAR_WALL_MAX_POSITION = 5;   // mm

//constexpr float MIN_ACCEPTABLE_COST = 1.5;
//constexpr float MAX_ACCEPTABLE_COST = 2;
constexpr float WEIGHT_MASS = 1/3;
constexpr float WEIGHT_NATURAL_FREQUENCIES = 1/3;
constexpr float WEIGHT_STRENGTH = 1/3;

constexpr unsigned SIZE_POPULATION = 20;
constexpr unsigned MAX_GENERATION_NUMBER = 500;
constexpr float PENALTY_R1 = 3;
constexpr float PENALTY_R2 = 2;
constexpr float PENALTY_BETA = 2;
constexpr float MUTATION_RATE = 0.02;

#endif /* Parameters_hpp */
