//
//  Parameters.hpp
//  BladeOptimization
//
//  Created by MyMac on 03.01.2022.
//

#ifndef Parameters_hpp
#define Parameters_hpp
//Main rotor
constexpr float MAIN_ROTOR_NOMINAL_SPEED = 1400; // rev/min
constexpr float MAIN_ROTOR_MIN_SPEED = 1000; // rev/min
constexpr float MAIN_ROTOR_MAX_SPEED = 1500; // rev/min

//Blade
constexpr float BLADE_LENGTH = 1100; // mm
constexpr float CHORD_LENGTH = 100; // mm
constexpr float PRETWIST_ANGLE = -7; // deg

constexpr float SAFETY_FACTOR_MIN = 1.5;
constexpr float SAFETY_FACTOR_MAX = 2;

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
constexpr unsigned BITS_PER_TM_XY = 3;

constexpr unsigned NUMBER_SPAR_LAYERS = 2;
constexpr unsigned BITS_PER_SPAR_LAYER = 6;
constexpr int SPAR_MIN_ANGLE = -89; // deg
constexpr int SPAR_MAX_ANGLE = 90;  // deg

constexpr unsigned BITS_PER_SPAR_WALL_ANGLE = 4;
constexpr int SPAR_WALL_MIN_ANGLE = -30; // deg
constexpr int SPAR_WALL_MAX_ANGLE = 30;  // deg

constexpr unsigned BITS_PER_SPAR_WALL_POSITION = 4;
constexpr int SPAR_WALL_MIN_POSITION = 2; // mm
constexpr int SPAR_WALL_MAX_POSITION = 5; // mm

//constexpr float MIN_ACCEPTABLE_COST = 1.5;
//constexpr float MAX_ACCEPTABLE_COST = 2;
//Cost functions parameters
constexpr float WEIGHT_MASS = 1./3;
constexpr float WEIGHT_NATURAL_FREQUENCIES = 1./3;
constexpr float WEIGHT_STRENGTH = 1./3;

constexpr float WEIGHT_SIGMA = 1./4;
constexpr float WEIGHT_UZ = 1./4;
constexpr float WEIGHT_UR = 1./4;
constexpr float WEIGHT_UTWIST = 1./4;

//Constraints - penalty functions parameters
//Trash values
constexpr float R_NF = 1;
constexpr float BETA_NF = 2;
constexpr float R_SF = 1;
constexpr float BETA_SF = 2;

constexpr float MAX_ACCEPTABLE_BIAS_TIP_FLAP = 5; // mm
//Trash values
constexpr float R_BTF = 1;
constexpr float BETA_BTF = 2;

constexpr float MAX_ACCEPTABLE_TWIST_ANGLE = 3; // deg
//Trash values
constexpr float R_TWIST = 1;
constexpr float BETA_TWIST = 2;

//Optimization
constexpr unsigned SIZE_POPULATION = 20;
constexpr unsigned MAX_GENERATION_NUMBER = 80;
constexpr float PENALTY_R1 = 3;
constexpr float PENALTY_R2 = 2;
constexpr float PENALTY_BETA = 2;
constexpr float MUTATION_RATE = 0.02;

#endif /* Parameters_hpp */
