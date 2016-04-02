// -*- C++ -*-
#ifndef V3D_NONLINEAR_LSQR_H
#define V3D_NONLINEAR_LSQR_H

#include "Math/v3d_linear.h"
#include "Math/v3d_linear_tnt.h"
#include "Math/v3d_mathutilities.h"
#include "Math/v3d_optimization.h"

#include <vector>
#include <iostream>

namespace V3D
{

#if defined(V3DLIB_ENABLE_SUITESPARSE)

#define NLSQ_MAX_PARAM_TYPES 32

   struct NLSQ_ParamDesc
   {
         int nParamTypes;                     //!< How many different kinds of parameters exist (2 for std. BA, cameras and 3D points)
         int dimension[NLSQ_MAX_PARAM_TYPES]; //!< What is the dimension of each parameter kind
         int count[NLSQ_MAX_PARAM_TYPES];     //!< How many unknowns are there per parameter type
   }; // end struct NLSQ_ParamDesc

   //! This structure holds the residuals, weights and Jacobian for a particular cost function.
   struct NLSQ_Residuals
   {
         NLSQ_Residuals(vector<int> const& usedParamTypes, int const nMeasurements,
                        int const measurementDimension, NLSQ_ParamDesc const& paramDesc)
            : _residuals(nMeasurements, measurementDimension),
              _weights(nMeasurements),
              _Js(usedParamTypes.size())
         {
            for (int k = 0; k < usedParamTypes.size(); ++k)
            {
               int const paramType = usedParamTypes[k];
               int const paramDimension = paramDesc.dimension[paramType];
               _Js[k] = new MatrixArray<double>(nMeasurements, measurementDimension, paramDimension);
            } // end for (k)
         } // end NLSQ_Residuals()

         ~NLSQ_Residuals()
         {
            for (int i = 0; i < _Js.size(); ++i) delete _Js[i];
         }

         VectorArray<double> _residuals;
         Vector<double>      _weights;

         vector<MatrixArray<double> * > _Js;
   }; // end struct NLSQ_Residuals

   struct NLSQ_LM_BlockHessian
   {
         NLSQ_LM_BlockHessian()
         {
            for (int t1 = 0; t1 < NLSQ_MAX_PARAM_TYPES; ++t1)
               for (int t2 = 0; t2 < NLSQ_MAX_PARAM_TYPES; ++t2)
                  Hs[t1][t2] = 0;
         }

         void allocateMatrices(NLSQ_ParamDesc const& paramDesc)
         {
            int const nParamTypes = paramDesc.nParamTypes;

            for (int t1 = 0; t1 < nParamTypes; ++t1)
               for (int t2 = 0; t2 < nParamTypes; ++t2)
               {
                  int const nz = nonzeroPairs[t1][t2].size();
                  if (nz > 0)
                  {
                     int const rows = paramDesc.dimension[t1];
                     int const cols = paramDesc.dimension[t2];
                     Hs[t1][t2] = new MatrixArray<double>(nz, rows, cols);
                  }
                  else
                     Hs[t1][t2] = 0;
               } // end for (t2)
         } // end allocateMatrices()

         void deallocateMatrices()
         {
            for (int t1 = 0; t1 < NLSQ_MAX_PARAM_TYPES; ++t1)
               for (int t2 = 0; t2 < NLSQ_MAX_PARAM_TYPES; ++t2)
                  if (Hs[t1][t2]) delete Hs[t1][t2];
         }

         void setZero()
         {
            for (int t1 = 0; t1 < NLSQ_MAX_PARAM_TYPES; ++t1)
               for (int t2 = 0; t2 < NLSQ_MAX_PARAM_TYPES; ++t2)
                  if (Hs[t1][t2])
                  {
                     MatrixArray<double>& H = *Hs[t1][t2];
                     for (int n = 0; n < H.count(); ++n) makeZeroMatrix(H[n]);
                  }
         } // end setZero()

         vector<pair<int, int> > nonzeroPairs[NLSQ_MAX_PARAM_TYPES][NLSQ_MAX_PARAM_TYPES];
         MatrixArray<double> *   Hs[NLSQ_MAX_PARAM_TYPES][NLSQ_MAX_PARAM_TYPES];
   }; // end struct NLSQ_LM_BlockHessian

   struct NLSQ_CostFunction
   {
         NLSQ_CostFunction(std::vector<int> const& usedParamTypes,
                           Matrix<int> const& correspondingParams,
                           int const measurementDimension)
            : _usedParamTypes(usedParamTypes),
              _nMeasurements(correspondingParams.num_rows()),
              _measurementDimension(measurementDimension),
              _correspondingParams(correspondingParams)
         {
            assert(usedParamTypes.size() == correspondingParams.num_cols());
         }

         virtual double getWeight(Vector<double> const& residual) const { return 1.0; }
         virtual void evalResidual(int const k, Vector<double>& residual) const = 0;

         virtual void initializeJacobian() { }
         virtual void fillJacobian(int const whichParam, int const paramIx, int const k, Matrix<double>& J) const = 0;

      protected:
         void evalAllResiduals(VectorArray<double>& residuals) const
         {
            for (int k = 0; k < _nMeasurements; ++k) this->evalResidual(k, residuals[k]);
         } // end evalAllResiduals()

         void fillAllWeights(VectorArray<double> const& residuals, Vector<double>& w) const
         {
            for (int k = 0; k < _nMeasurements; ++k) w[k] = this->getWeight(residuals[k]);
         }

         void fillAllJacobians(Vector<double> const& weights, vector<MatrixArray<double> * >& Js) const;

         std::vector<int> const& _usedParamTypes;
         int              const  _nMeasurements;
         int              const  _measurementDimension;
         Matrix<int>      const& _correspondingParams;

         friend struct NLSQ_LM_Optimizer;
   }; // end struct NLSQ_CostFunction

   struct NLSQ_LM_Optimizer : public LevenbergOptimizerCommon
   {
         NLSQ_LM_Optimizer(NLSQ_ParamDesc const& paramDesc,
                           std::vector<NLSQ_CostFunction *> const& costFunctions);

         ~NLSQ_LM_Optimizer();

         void minimize();

         virtual double getParameterLength() const = 0;

         virtual void updateParameters(int const paramType, VectorArrayAdapter<double> const& delta) = 0;
         virtual void saveAllParameters() = 0;
         virtual void restoreAllParameters() = 0;

         CCS_Matrix<double> const& JtJ() const { return _JtJ; }

      protected:
         void setupSparseJtJ();
         void fillJtJ();
         void fillJacobians();
         void fillHessian();
         void evalJt_e(Vector<double>& Jt_e);

         //! Map (paramType,ix) pairs to a continuous range of global ids.
         int getParamId(int const paramType, int const paramIx) const { return _paramTypeStartID[paramType] + paramIx; }

         NLSQ_ParamDesc                   const& _paramDesc;
         std::vector<NLSQ_CostFunction *> const& _costFunctions;

         //! Mapping of parameter types to their first id in the continuous range.
         int _paramTypeStartID[NLSQ_MAX_PARAM_TYPES + 1];
         int _totalParamCount;
         vector<pair<int, int> > _paramIdInverseMap; //!< the reverse mapping from global ids to (paramType,id) pairs.

         vector<NLSQ_Residuals *>   _residuals;      // one per cost function
         //!< _hessianIndices establishes the link between the local hessian in a cost function to the global non-zero block.
         vector<MatrixArray<int> *> _hessianIndices; // one per cost function
         NLSQ_LM_BlockHessian       _hessian;

         // Used for sparse Cholesky
         std::vector<int> _JtJ_Lp, _JtJ_Parent, _JtJ_Lnz;
         std::vector<int> _perm_JtJ, _invPerm_JtJ;

         int _paramTypeRowStart[NLSQ_MAX_PARAM_TYPES + 1];
         CCS_Matrix<double> _JtJ;
   }; // end struct NLSQ_LM_Optimizer

#endif // defined(V3DLIB_ENABLE_SUITESPARSE)

} // end namespace V3D

#endif
