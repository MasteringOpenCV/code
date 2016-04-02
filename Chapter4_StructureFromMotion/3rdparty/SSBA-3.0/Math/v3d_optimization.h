// -*- C++ -*-

#ifndef V3D_OPTIMIZATION_H
#define V3D_OPTIMIZATION_H

#include "Math/v3d_linear.h"
#include "Math/v3d_linear_tnt.h"
#include "Math/v3d_mathutilities.h"

#include <vector>
#include <iostream>

namespace V3D
{

   enum
   {
      LEVENBERG_OPTIMIZER_TIMEOUT = 0,
      LEVENBERG_OPTIMIZER_SMALL_UPDATE = 1,
      LEVENBERG_OPTIMIZER_CONVERGED = 2
   };

   extern int optimizerVerbosenessLevel;

   struct LevenbergOptimizerCommon
   {
         LevenbergOptimizerCommon()
            : status(LEVENBERG_OPTIMIZER_TIMEOUT), currentIteration(0), maxIterations(50), minIterations(10),
              tau(1e-3), lambda(1e-3), gradientThreshold(1e-8), updateThreshold(1e-8), improvementThreshold(1e-8),
              _nu(2.0)
         { }

         virtual ~LevenbergOptimizerCommon()
         { }

#if 0
         // See Madsen et al., "Methods for non-linear least squares problems."
         virtual void increaseLambda()
         {
            lambda *= _nu; _nu *= 2.0;
         }

         virtual void decreaseLambda(double const rho)
         {
            double const r = 2*rho - 1.0;
            lambda *= std::max<double>(1.0/3.0, 1 - r*r*r);
            if (lambda < 1e-10) lambda = 1e-10;
            _nu = 2;
         }
#else
         virtual void increaseLambda()                 { lambda *= 10.0; }
         virtual void decreaseLambda(double const rho) { lambda = std::max(1e-10, lambda * 0.1); }
#endif

         bool applyGradientStoppingCriteria(double maxGradient) const
         {
            return maxGradient < gradientThreshold;
         }

         bool applyUpdateStoppingCriteria(double paramLength, double updateLength) const
         {
            if (currentIteration < minIterations) return false;
            return updateLength < updateThreshold * (paramLength + updateThreshold);
         }

         bool applyImprovementStoppingCriteria(double const deltaError, double const oldError) const
         {
            double const relImprovement = fabs(deltaError / oldError);
            return relImprovement < improvementThreshold;
         }

         int    status;
         int    currentIteration, maxIterations, minIterations;
         double tau, lambda;
         double gradientThreshold, updateThreshold, improvementThreshold;

      protected:
         double _nu;
   }; // end struct LevenbergOptimizerCommon

   //! Dense (and simple) Levenberg-Marquardt optimizer.
   struct SimpleLevenbergOptimizer : public LevenbergOptimizerCommon
   {
         SimpleLevenbergOptimizer(unsigned nMeasurements_, unsigned nParameters_)
            : LevenbergOptimizerCommon(),
              nMeasurements(nMeasurements_), nParameters(nParameters_),
              numDiffDelta(nParameters_, 1e-6),
              observation(nMeasurements),
              currentParameters(nParameters_), savedParameters(nParameters_)
         {
            makeZeroVector(observation);
         }

         virtual ~SimpleLevenbergOptimizer() { }

         //! Minimize the objective function starting from the current parameters.
         void minimize()
         {
            using namespace std;

            status = LEVENBERG_OPTIMIZER_TIMEOUT;
            bool computeDerivatives = true;

            Matrix<double> J(nMeasurements, nParameters);
            Matrix<double> JtJ(nParameters, nParameters);
            Matrix<double> N(nParameters, nParameters);
            Vector<double> e(nMeasurements), e_new(nMeasurements), Jt_e(nParameters);
            Vector<double> delta(nParameters);
            Vector<double> weights(nMeasurements);
            fillVector(1.0, weights);

            for (currentIteration = 0; currentIteration < maxIterations; ++currentIteration)
            {
               if (optimizerVerbosenessLevel >= 2) cout << "currentIteration: " << currentIteration << endl;

               this->fillWeights(weights);
               this->evalWeightedResidual(weights, e);
               double const err = sqrNorm_L2(e);

               if (optimizerVerbosenessLevel >= 1) cout << "residual = " << err << endl;
               if (optimizerVerbosenessLevel >= 2) cout << "lambda = " << lambda << endl;

               if (computeDerivatives)
               {
                  fillJacobian(J);
                  multiply_At_A(J, JtJ);
                  multiply_At_v(J, e, Jt_e);
               }

               if (this->applyGradientStoppingCriteria(norm_Linf(Jt_e)))
               {
                  status = LEVENBERG_OPTIMIZER_CONVERGED;
                  return;
               }

               if (currentIteration == 0)
               {
                  // Initialize lambda as tau*max(JtJ(i, i))
                  // Note: operator()(i, j) is 1-based.
                  double maxEl = -1e30;
                  for (unsigned l = 0; l < nParameters; ++l)
                     maxEl = std::max<double>(maxEl, JtJ[l][l]);

                  lambda = tau * maxEl;
                  if (optimizerVerbosenessLevel >= 2) cout << "initial lambda = " << lambda << endl;
               } // end if (currentIteration == 0)

               N = JtJ;
               for (unsigned k = 1; k <= N.num_rows(); ++k)
                  this->augmentDiagonal(N(k, k));

               Cholesky<double> chol(N);
               delta = chol.solve(Jt_e);

               if (optimizerVerbosenessLevel >= 2) cout << " deltaSqrLength = " << sqrNorm_L2(delta) << endl;

               double const paramLength = this->getParameterLength();
               if (this->applyUpdateStoppingCriteria(paramLength, norm_L2(delta)))
               {
                  status = LEVENBERG_OPTIMIZER_SMALL_UPDATE;
                  return;
               }

               this->saveCurrentParameters();
               updateCurrentParameters(delta);

               evalWeightedResidual(weights, e_new);
               double const err_new = sqrNorm_L2(e_new);
               double rho = err - err_new;

               if (optimizerVerbosenessLevel >= 2) cout << "new residual = " << err_new << " rho = " << rho << endl;

               {
                  double denom1 = sqrNorm_L2(delta);
                  double denom2 = innerProduct(delta, Jt_e);

                  denom1 *= lambda;
                  rho = rho / (denom1 + denom2);
                  if (optimizerVerbosenessLevel >= 2) cout << " rho = " << rho << endl;
               }

               if (rho > 0)
               {
                  // Improvement in the new solution
                  decreaseLambda(rho);
                  computeDerivatives = true;
               }
               else
               {
                  restoreCurrentParameters();
                  increaseLambda();
                  computeDerivatives = false;
               } // end if (rho > 0)
            } // end for
         } // end minimize()

         //! Minimize the objective function using from the given parameters.
         void minimize(Vector<double> const& startingParameters)
         {
            currentParameters = startingParameters;
            minimize();
         }

         //virtual void augmentDiagonal(double& v) const { v *= (1.0 + lambda); }
         virtual void augmentDiagonal(double& v) const { v += lambda; }

         virtual double getParameterLength() const
         {
            return norm_L2(currentParameters);
         }

         virtual void updateCurrentParameters(Vector<double> const& delta)
         {
            currentParameters += delta;
         }

         virtual void saveCurrentParameters()
         {
            savedParameters = currentParameters;
         }

         virtual void restoreCurrentParameters()
         {
            currentParameters = savedParameters;
         }

         //! residual = observation - f(currentParameters)
         virtual void evalWeightedResidual(Vector<double> const& w, Vector<double>& res)
         {
            evalFunction(res);
            for (unsigned i = 0; i < res.size(); ++i)
               res[i] = w[i] * (observation[i] - res[i]);
         }

         virtual void evalFunction(Vector<double>& res) = 0;
         virtual void fillJacobian(Matrix<double>& J) { this->approximateJacobian(J); }

         virtual void fillWeights(Vector<double>& w)
         {
            std::fill(w.begin(), w.end(), 1.0);
         }

         //! Approximates the Jacobian using a 2-sided difference scheme.
         void approximateJacobian(Matrix<double>& J)
         {
            Vector<double> update(nParameters);
            makeZeroVector(update);
            Vector<double> y1(nMeasurements), y2(nMeasurements);

            for (unsigned j = 0; j < J.num_cols(); ++j)
            {
               saveCurrentParameters();

               update[j] = -numDiffDelta[j];
               updateCurrentParameters(update);
               evalFunction(y1);
               restoreCurrentParameters();

               update[j] = numDiffDelta[j];
               updateCurrentParameters(update);
               evalFunction(y2);

               // Note: operator()(i, j) is 1-based.
               for (unsigned i = 0; i < J.num_rows(); ++i)
                  J(1+i, 1+j) = (y2[i] - y1[i])/2/numDiffDelta[j];

               update[j] = 0;
               restoreCurrentParameters();
            } // end for (j)
         }

         unsigned nMeasurements, nParameters;
         std::vector<double> numDiffDelta;

         Vector<double> observation;
         Vector<double> currentParameters, savedParameters;
   }; // end struct SimpleLevenbergOptimizer

//----------------------------------------------------------------------

# if defined(V3DLIB_ENABLE_SUITESPARSE)

   struct SparseLevenbergOptimizer : public LevenbergOptimizerCommon
   {
         SparseLevenbergOptimizer(int measurementDimension,
                                  int nParametersA, int paramDimensionA,
                                  int nParametersB, int paramDimensionB,
                                  int paramDimensionC,
                                  std::vector<int> const& correspondingParamA,
                                  std::vector<int> const& correspondingParamB)
            : LevenbergOptimizerCommon(),
              _nMeasurements(correspondingParamA.size()),
              _measurementDimension(measurementDimension),
              _nParametersA(nParametersA), _paramDimensionA(paramDimensionA),
              _nParametersB(nParametersB), _paramDimensionB(paramDimensionB),
              _paramDimensionC(paramDimensionC),
              _nNonvaryingA(0), _nNonvaryingB(0), _nNonvaryingC(0),
              _correspondingParamA(correspondingParamA),
              _correspondingParamB(correspondingParamB)
         {
            assert(correspondingParamA.size() == correspondingParamB.size());
         }

         ~SparseLevenbergOptimizer() { }

         void setNonvaryingCounts(int nNonvaryingA, int nNonvaryingB, int nNonvaryingC)
         {
            _nNonvaryingA = nNonvaryingA;
            _nNonvaryingB = nNonvaryingB;
            _nNonvaryingC = nNonvaryingC;
         }

         void getNonvaryingCounts(int& nNonvaryingA, int& nNonvaryingB, int& nNonvaryingC) const
         {
            nNonvaryingA = _nNonvaryingA;
            nNonvaryingB = _nNonvaryingB;
            nNonvaryingC = _nNonvaryingC;
         }

         void minimize();

         virtual void evalResidual(VectorArray<double>& residuals) = 0;

         virtual void fillWeights(VectorArray<double> const& residuals, Vector<double>& w)
         {
            std::fill(w.begin(), w.end(), 1.0);
         }

         void fillAllJacobians(Vector<double> const& w,
                               MatrixArray<double>& Ak,
                               MatrixArray<double>& Bk,
                               MatrixArray<double>& Ck)
         {
            int const nVaryingA = _nParametersA - _nNonvaryingA;
            int const nVaryingB = _nParametersB - _nNonvaryingB;
            int const nVaryingC = _paramDimensionC - _nNonvaryingC;

            for (int k = 0; k < _nMeasurements; ++k)
            {
               int const i = _correspondingParamA[k];
               int const j = _correspondingParamB[k];

               if (i < _nNonvaryingA && j < _nNonvaryingB) continue;

               fillJacobians(Ak[k], Bk[k], Ck[k], i, j, k);

               // Clear the Jacobians for locked parameters.
               if (i < _nNonvaryingA) makeZeroMatrix(Ak[k]);
               if (j < _nNonvaryingB) makeZeroMatrix(Bk[k]);

               if (_nNonvaryingC > 0)
               {
                  for (int r = 0; r < _measurementDimension; ++r)
                     for (int c = 0; c < _nNonvaryingC; ++c) Ck[k][r][c] = 0.0;
               }
            } // end for (k)

            if (nVaryingA > 0)
            {
               for (int k = 0; k < _nMeasurements; ++k)
                  scaleMatrixIP(w[k], Ak[k]);
            }
            if (nVaryingB > 0)
            {
               for (int k = 0; k < _nMeasurements; ++k)
                  scaleMatrixIP(w[k], Bk[k]);
            }
            if (nVaryingC > 0)
            {
               for (int k = 0; k < _nMeasurements; ++k)
                  scaleMatrixIP(w[k], Ck[k]);
            }
         } // end fillAllJacobians()

         virtual void setupJacobianGathering() { }

         virtual void fillJacobians(Matrix<double>& Ak, Matrix<double>& Bk, Matrix<double>& Ck,
                                    int i, int j, int k) = 0;

         virtual double getParameterLength() const = 0;

         virtual void updateParametersA(VectorArray<double> const& deltaAi) = 0;
         virtual void updateParametersB(VectorArray<double> const& deltaBj) = 0;
         virtual void updateParametersC(Vector<double> const& deltaC) = 0;
         virtual void saveAllParameters() = 0;
         virtual void restoreAllParameters() = 0;

         CCS_Matrix<double> const& JtJ() const
         {
            return _JtJ;
         }

      protected:
         void serializeNonZerosJtJ(std::vector<std::pair<int, int> >& dst) const;
         void setupSparseJtJ();
         void fillSparseJtJ(MatrixArray<double> const& Ui, MatrixArray<double> const& Vj, MatrixArray<double> const& Wk,
                            Matrix<double> const& Z, Matrix<double> const& X, Matrix<double> const& Y);

         int const _nMeasurements, _measurementDimension;
         int const _nParametersA, _paramDimensionA;
         int const _nParametersB, _paramDimensionB;
         int const _paramDimensionC;

         int _nNonvaryingA, _nNonvaryingB, _nNonvaryingC;

         std::vector<int> const& _correspondingParamA;
         std::vector<int> const& _correspondingParamB;

         std::vector<pair<int, int> > _jointNonzerosW;
         std::vector<int>             _jointIndexW;

         std::vector<int> _JtJ_Lp, _JtJ_Parent, _JtJ_Lnz;
         std::vector<int> _perm_JtJ, _invPerm_JtJ;

         CCS_Matrix<double> _JtJ;
   }; // end struct SparseLevenbergOptimizer

//----------------------------------------------------------------------

   struct StdSparseLevenbergOptimizer : public SparseLevenbergOptimizer
   {
         StdSparseLevenbergOptimizer(int measurementDimension,
                                     int nParametersA, int paramDimensionA,
                                     int nParametersB, int paramDimensionB,
                                     int paramDimensionC,
                                     std::vector<int> const& correspondingParamA,
                                     std::vector<int> const& correspondingParamB)
            : SparseLevenbergOptimizer(measurementDimension, nParametersA, paramDimensionA,
                                       nParametersB, paramDimensionB, paramDimensionC,
                                       correspondingParamA, correspondingParamB),
              curParametersA(nParametersA, paramDimensionA), savedParametersA(nParametersA, paramDimensionA),
              curParametersB(nParametersB, paramDimensionB), savedParametersB(nParametersB, paramDimensionB),
              curParametersC(paramDimensionC), savedParametersC(paramDimensionC)
         { }

         virtual double getParameterLength() const
         {
            double res = 0.0;
            for (int i = 0; i < _nParametersA; ++i) res += sqrNorm_L2(curParametersA[i]);
            for (int j = 0; j < _nParametersB; ++j) res += sqrNorm_L2(curParametersB[j]);
            res += sqrNorm_L2(curParametersC);
            return sqrt(res);
         }

         virtual void updateParametersA(VectorArray<double> const& deltaAi)
         {
            for (int i = 0; i < _nParametersA; ++i) addVectors(deltaAi[i], curParametersA[i], curParametersA[i]);
         }

         virtual void updateParametersB(VectorArray<double> const& deltaBj)
         {
            for (int j = 0; j < _nParametersB; ++j) addVectors(deltaBj[j], curParametersB[j], curParametersB[j]);
         }

         virtual void updateParametersC(Vector<double> const& deltaC)
         {
            addVectors(deltaC, curParametersC, curParametersC);
         }

         virtual void saveAllParameters()
         {
            for (int i = 0; i < _nParametersA; ++i) savedParametersA[i] = curParametersA[i];
            for (int j = 0; j < _nParametersB; ++j) savedParametersB[j] = curParametersB[j];
            savedParametersC = curParametersC;
         }

         virtual void restoreAllParameters()
         {
            for (int i = 0; i < _nParametersA; ++i) curParametersA[i] = savedParametersA[i];
            for (int j = 0; j < _nParametersB; ++j) curParametersB[j] = savedParametersB[j];
            curParametersC = savedParametersC;
         }

         VectorArray<double> curParametersA, savedParametersA;
         VectorArray<double> curParametersB, savedParametersB;
         Vector<double>      curParametersC, savedParametersC;
   }; // end struct StdSparseLevenbergOptimizer

//----------------------------------------------------------------------

   struct SparseLM_ParameterInfo
   {
         SparseLM_ParameterInfo(int nParametersA_, int paramDimensionA_,
                                int nParametersB_, int paramDimensionB_,
                                int paramDimensionC_)
            : nParametersA(nParametersA_), paramDimensionA(paramDimensionA_),
              nParametersB(nParametersB_), paramDimensionB(paramDimensionB_),
              paramDimensionC(paramDimensionC_),
              nNonvaryingA(0), nNonvaryingB(0), nNonvaryingC(0)
         {
            nVaryingA = nParametersA - nNonvaryingA;
            nVaryingB = nParametersB - nNonvaryingB;
            nVaryingC = paramDimensionC - nNonvaryingC;
         }

         void setNonvaryingCounts(int const nNonvaryingA_, int const nNonvaryingB_, int const nNonvaryingC_)
         {
            nNonvaryingA = nNonvaryingA_;
            nNonvaryingB = nNonvaryingB_;
            nNonvaryingC = nNonvaryingC_;
            nVaryingA = nParametersA - nNonvaryingA;
            nVaryingB = nParametersB - nNonvaryingB;
            nVaryingC = paramDimensionC - nNonvaryingC;
         }

         void getNonvaryingCounts(int& nNonvaryingA_, int& nNonvaryingB_, int& nNonvaryingC_) const
         {
            nNonvaryingA_ = nNonvaryingA;
            nNonvaryingB_ = nNonvaryingB;
            nNonvaryingC_ = nNonvaryingC;
         }

         int const nParametersA, paramDimensionA;
         int const nParametersB, paramDimensionB;
         int const paramDimensionC;

         int nNonvaryingA, nNonvaryingB, nNonvaryingC;
         int nVaryingA, nVaryingB, nVaryingC;
   }; // end struct SparseLM_ParameterInfo

   struct SparseLM_CostFunction
   {
         SparseLM_CostFunction(SparseLM_ParameterInfo const& paramInfo,
                               int const measurementDimension,
                               std::vector<int> const& correspondingParamA,
                               std::vector<int> const& correspondingParamB)
            : _paramInfo(paramInfo),
              _nMeasurements(correspondingParamA.size()),
              _measurementDimension(measurementDimension),
              _correspondingParamA(correspondingParamA),
              _correspondingParamB(correspondingParamB),
              _residuals(_nMeasurements, measurementDimension),
              _weights(_nMeasurements),
              _Ak(_nMeasurements, measurementDimension, paramInfo.paramDimensionA),
              _Bk(_nMeasurements, measurementDimension, paramInfo.paramDimensionB),
              _Ck(_nMeasurements, measurementDimension, paramInfo.paramDimensionC)
         {
            assert(correspondingParamA.size() == correspondingParamB.size());
         }

         ~SparseLM_CostFunction() { }

         // Domain specific code to be defined in  derived classes
         virtual void fillWeights(VectorArray<double> const& residuals, Vector<double>& w) const
         {
            std::fill(w.begin(), w.end(), 1.0);
         }
         virtual void evalResidual(VectorArray<double>& residuals) const = 0;
         virtual void setupJacobianGathering() { }
         virtual void fillJacobians(Matrix<double>& Ak, Matrix<double>& Bk, Matrix<double>& Ck,
                                    int i, int j, int k) const = 0;
         // End of domain specific code

      protected:
         void fillAllJacobians();
         void fillHessian(std::vector<int> const& jointIndexW,
                          MatrixArray<double>& Ui, MatrixArray<double>& Vj,
                          MatrixArray<double>& Wn,
                          Matrix<double>& X, Matrix<double>& Y, Matrix<double>& Z);
         void evalJt_e(VectorArray<double>& At_e, VectorArray<double>& Bt_e, Vector<double>& Ct_e);

         SparseLM_ParameterInfo const _paramInfo;
         int const _nMeasurements, _measurementDimension;

         std::vector<int> const& _correspondingParamA;
         std::vector<int> const& _correspondingParamB;

         VectorArray<double> _residuals;
         Vector<double> _weights;
         MatrixArray<double> _Ak, _Bk, _Ck;

         friend struct ExtSparseLevenbergOptimizer;
   }; // end struct SparseLM_CostFunction

   struct ExtSparseLevenbergOptimizer : public LevenbergOptimizerCommon
   {
         ExtSparseLevenbergOptimizer(SparseLM_ParameterInfo const& paramInfo,
                                     std::vector<SparseLM_CostFunction *> const& costFunctions)
            : LevenbergOptimizerCommon(),
              _paramInfo(paramInfo), _costFunctions(costFunctions),
              _Ui(paramInfo.nVaryingA, paramInfo.paramDimensionA, paramInfo.paramDimensionA),
              _Vj(paramInfo.nVaryingB, paramInfo.paramDimensionB, paramInfo.paramDimensionB),
              _X(paramInfo.nVaryingA*paramInfo.paramDimensionA, paramInfo.nVaryingC),
              _Y(paramInfo.nVaryingB*paramInfo.paramDimensionB, paramInfo.nVaryingC),
              _Z(paramInfo.nVaryingC, paramInfo.nVaryingC)
         { }

         ~ExtSparseLevenbergOptimizer() { }

         void minimize();

         virtual double getParameterLength() const = 0;

         virtual void updateParametersA(VectorArray<double> const& deltaAi) = 0;
         virtual void updateParametersB(VectorArray<double> const& deltaBj) = 0;
         virtual void updateParametersC(Vector<double> const& deltaC) = 0;
         virtual void saveAllParameters() = 0;
         virtual void restoreAllParameters() = 0;

         CCS_Matrix<double> const& JtJ() const { return _JtJ; }

      protected:
         void setupSparseJtJ();
         void serializeNonZerosJtJ(std::vector<std::pair<int, int> >& dst) const;
         void fillJtJ(MatrixArray<double> const& Ui, MatrixArray<double> const& Vj, MatrixArray<double> const& Wn,
                      Matrix<double> const& Z, Matrix<double> const& X, Matrix<double> const& Y);

         SparseLM_ParameterInfo               const& _paramInfo;
         std::vector<SparseLM_CostFunction *> const& _costFunctions;

         std::vector<pair<int, int> >   _jointNonzerosW; // vector of all pairs (i,j) generated by all measurements
         std::vector<std::vector<int> > _jointIndexW; // maps measurements wrt. the cost function to the position of Wij

         std::vector<int> _JtJ_Lp, _JtJ_Parent, _JtJ_Lnz;
         std::vector<int> _perm_JtJ, _invPerm_JtJ;

         CCS_Matrix<double> _JtJ;
         MatrixArray<double> _Ui, _Vj;
         Matrix<double> _X, _Y, _Z; // X = A^t*C, Y = B^t*C, Z = C^t*C
   }; // end struct ExtSparseLevenbergOptimizer

# endif // defined(V3DLIB_ENABLE_SUITESPARSE)

//**********************************************************************

   enum ConstraintType
      {
         LP_EQUAL,
         LP_LESS_EQUAL,
         LP_GREATER_EQUAL
      };

   enum LP_SolverType
      {
         LP_LPSOLVE55,
         LP_DSDP
      };

   struct LP_Configuration
   {
         LP_Configuration()
            : solverType(LP_LPSOLVE55),
              verbose(false), maximize(false), useInitialValue(false)
         { }

         LP_SolverType solverType;
         bool verbose, maximize, useInitialValue;
   }; // end struct LP_Configuration

   bool solveLinearProgram(std::vector<double> const& costs,
                           CCS_Matrix<double> const& A,
                           std::vector<ConstraintType> const& constraintTypes,
                           std::vector<double> const& b,
                           std::vector<std::pair<int, double> > const& lowerBounds,
                           std::vector<std::pair<int, double> > const& upperBounds,
                           std::vector<int> const& nonNegativeVariables,
                           std::vector<double>& result, LP_Configuration const& conf = LP_Configuration());

} // end namespace V3D

#endif
