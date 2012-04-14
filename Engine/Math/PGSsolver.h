#ifndef INCLUDE_PGSSOLVER
#define INCLUDE_PGSSOLVER

#include <vector>
using namespace std;

void SolveLambda(const vector<vector<double>> &matr, const vector<double> &eta,
      vector<double> &lambda, double lambdaMin, double lambdaMax);

#endif