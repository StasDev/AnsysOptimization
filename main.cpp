#include "Optimization.hpp"

int main(int argc, const char * argv[]) {
//    Simple tests
//    ARRAY_ANGLES arrAngles;
//    std::cout << arrAngles;
//    arrAngles.SetApproxAngle(-23, 2);
//    std::cout << arrAngles;
//    arrAngles.WriteAnglesInFileWithPath();
    
//    Test launch python file
//    Py_Initialize();
//    PyRun_SimpleString("print('Test')");
//    FILE* fp = fopen("/Users/stanislavdumanskij/Documents/GraphiteWork_small/Projects/HelicopterMianRotor/BladeOptimizationCpp/BladeOptimizationCpp/PythonFiles/test.py", "r");
//    PyRun_AnyFile(fp, "/Users/stanislavdumanskij/Documents/GraphiteWork_small/Projects/HelicopterMianRotor/BladeOptimizationCpp/BladeOptimizationCpp/PythonFiles/test.py");
//    Py_Finalize();
    
    SIMPLE_GA test(SIZE_POPULATION,MAX_GENERATION_NUMBER, MIN_ACCEPTABLE_COST,MAX_ACCEPTABLE_COST, MUTATION_RATE);
    return 0;
}
