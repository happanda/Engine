#include "PGSsolver.h"
#include <assert.h>
#include <math.h>
#include "Math\MathRoutines.h"

void SolveLambda(const vector<vector<double>> &matr, const vector<double> &eta,
      vector<double> &lambda, double lambdaMin, double lambdaMax)
{
   assert (lambda.size() == eta.size());
   assert (lambda.size() == matr.size());
   size_t n = lambda.size();
   for (size_t i = 0; i < n; i++)
   {
      assert (lambda.size() == matr[i].size());
      assert (matr[i][i] != 0);
   }
   for (size_t i = 0; i < n; i++)
   {
      double val = eta[i];
      for (size_t j = 0; j < n; j++)
         val -= matr[i][j] * lambda[j];
      lambda[i] = clamp(val / matr[i][i], lambdaMin, lambdaMax);
   }
}