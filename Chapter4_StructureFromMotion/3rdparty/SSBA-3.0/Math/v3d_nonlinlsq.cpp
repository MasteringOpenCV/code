#include "Math/v3d_nonlinlsq.h"

#include <map>

#if defined(V3DLIB_ENABLE_SUITESPARSE)
# include "colamd.h"
# include "Math/v3d_ldl_private.h"

using namespace std;
using namespace V3D;

#define ONLY_UPPER_TRIANGULAR_HESSIAN 1

namespace V3D
{

   void
   NLSQ_CostFunction::fillAllJacobians(Vector<double> const& weights, vector<MatrixArray<double> * >& Js) const
   {
      for (int i = 0; i < _usedParamTypes.size(); ++i)
      {
         MatrixArray<double>& J = *Js[i];
         for (int k = 0; k < _nMeasurements; ++k)
         {
            int const paramIx = _correspondingParams[k][i];
            this->fillJacobian(i, paramIx, k, J[k]);
            scaleMatrixIP(weights[k], J[k]);
         } // end for (k)
      } // end for (i)
   } // end NLSQ_CostFunction::fillAllJacobians()

//======================================================================

   NLSQ_LM_Optimizer::NLSQ_LM_Optimizer(NLSQ_ParamDesc const& paramDesc,
                                        std::vector<NLSQ_CostFunction *> const& costFunctions)
      : LevenbergOptimizerCommon(),
        _paramDesc(paramDesc), _costFunctions(costFunctions),
        _hessianIndices(costFunctions.size()), _residuals(costFunctions.size())
   {
      // First, compute the mappings between (paramType, id) pairs and global continuous ids.
      _paramTypeStartID[0] = 0;
      for (int paramType = 0; paramType < paramDesc.nParamTypes; ++paramType)
         _paramTypeStartID[paramType+1] = _paramTypeStartID[paramType] + paramDesc.count[paramType];

      _totalParamCount = _paramTypeStartID[paramDesc.nParamTypes];

      _paramIdInverseMap.resize(_totalParamCount);
      for (int paramType = 0; paramType < paramDesc.nParamTypes; ++paramType)
         for (int ix = 0; ix < paramDesc.count[paramType]; ++ix)
         {
            int const id = this->getParamId(paramType, ix);
            _paramIdInverseMap[id].first = paramType;
            _paramIdInverseMap[id].second = ix;
         }

      // This does the hard work of setting up the sparse data structures.
      this->setupSparseJtJ();
   } // end NLSQ_LM_Optimizer()

   NLSQ_LM_Optimizer::~NLSQ_LM_Optimizer()
   {
      _hessian.deallocateMatrices();

      for (int obj = 0; obj < _costFunctions.size(); ++obj)
      {
         delete _hessianIndices[obj];
         delete _residuals[obj];
      }
   } // end NLSQ_LM_Optimizer::~NLSQ_LM_Optimizer()

   void
   NLSQ_LM_Optimizer::setupSparseJtJ()
   {
      int const nObjs = _costFunctions.size();

      // Note: for logical parameter ids i and j, J^T J has a non-zero block at (i,j) iff
      // there exists at least one measurement depending on i and j.

      typedef map<pair<int, int>, int > NonzeroPosMap;

      Matrix<NonzeroPosMap> nonzeroPosMaps(NLSQ_MAX_PARAM_TYPES, NLSQ_MAX_PARAM_TYPES);
      int nNonzeroBlocks = 0;

      // Found out which non-zero blocks exist in the Hessian.
      for (int obj = 0; obj < nObjs; ++obj)
      {
         NLSQ_CostFunction& costFun = *_costFunctions[obj];

         int const nParamTypes = costFun._usedParamTypes.size();
         int const nMeasurements = costFun._nMeasurements;

         _hessianIndices[obj] = new MatrixArray<int>(nMeasurements, nParamTypes, nParamTypes);
         MatrixArray<int>& index = *_hessianIndices[obj];

         // Also allocate the residual object here
         _residuals[obj] = new NLSQ_Residuals(costFun._usedParamTypes, nMeasurements,
                                              costFun._measurementDimension, _paramDesc);

         for (int k = 0; k < nMeasurements; ++k)
         {
            for (int i1 = 0; i1 < nParamTypes; ++i1)
            {
               int const t1  = costFun._usedParamTypes[i1];
               int const ix1 = costFun._correspondingParams[k][i1];
               int const id1 = this->getParamId(t1, ix1);

               for (int i2 = 0; i2 < nParamTypes; ++i2)
               {
                  int const t2  = costFun._usedParamTypes[i2];
                  int const ix2 = costFun._correspondingParams[k][i2];
                  int const id2 = this->getParamId(t2, ix2);

#if defined(ONLY_UPPER_TRIANGULAR_HESSIAN)
                  if (id1 > id2) continue; // only store the upper diagonal blocks
#endif

                  NonzeroPosMap& nzPosMap = nonzeroPosMaps[t1][t2];
                  NonzeroPosMap::const_iterator p = nzPosMap.find(make_pair(ix1, ix2));

                  if (p == nzPosMap.end())
                  {
                     // We have a new non-zero block that needs to be stored in the Hessian.
                     int const curPos = nzPosMap.size();
                     index[k][i1][i2] = curPos;
                     nzPosMap.insert(make_pair(make_pair(ix1, ix2), curPos));
                     _hessian.nonzeroPairs[t1][t2].push_back(make_pair(ix1, ix2));
                     ++nNonzeroBlocks;
                  }
                  else
                  {
                     index[k][i1][i2] = p->second;
                  } // end if
               } // end for (i2)
            } // end for (i1)
         } // end for (k)
      } // end for (obj)

      _hessian.allocateMatrices(_paramDesc);
      if (0) {
         cout << "nNonzeroBlocks = " << nNonzeroBlocks << endl;
         int const nParamTypes = _paramDesc.nParamTypes;
         for (int t1 = 0; t1 < nParamTypes; ++t1)
            for (int t2 = 0; t2 < nParamTypes; ++t2)
            {
               cout << "Hs[" << t1 << "][" << t2 << "] = " << _hessian.Hs[t1][t2] << endl;
               if (_hessian.Hs[t1][t2])
                  cout << "Hs.size() = " << _hessian.Hs[t1][t2]->count() << endl;
               cout << "_hessian.nonzeroPairs[t1][t2].size() = " << _hessian.nonzeroPairs[t1][t2].size() << endl;
               vector<pair<int, int> > const& pairs = _hessian.nonzeroPairs[t1][t2];
               //for (int k = 0; k < pairs.size(); ++k) cout << " " << pairs[k].first << " <-> " << pairs[k].second << endl;
            }
      }

      // At thos point we know the (block) nonzeros of J^T J and we have allocated the needed memory.
      // Now for the column reordering

      // For the column reordering we treat the columns belonging to one set
      // of parameters as one (logical) column.

      // Determine non-zeros of JtJ (we forget about the non-zero diagonal for now)
      // Also ignore the upper triangular part of JtJ
      // Only consider nonzeros of Ai^t * Bj induced by the measurements.
      vector<pair<int, int> > nz_blockJtJ;
      nz_blockJtJ.reserve(nNonzeroBlocks);
      for (int t1 = 0; t1 < _paramDesc.nParamTypes; ++t1)
         for (int t2 = 0; t2 < _paramDesc.nParamTypes; ++t2)
         {
            vector<pair<int, int> > const& nz = _hessian.nonzeroPairs[t1][t2];
            for (int k = 0; k < nz.size(); ++k)
            {
               int const ix1 = nz[k].first;
               int const ix2 = nz[k].second;
               int const id1 = this->getParamId(t1, ix1);
               int const id2 = this->getParamId(t2, ix2);
               // Note: id1 < id2 (lower triangular part), but for symamd one
               // needs only the strictly lower triangular part.
               if (id1 != id2) nz_blockJtJ.push_back(make_pair(id2, id1));
            } // end for (t1)
         } // end for (t2)

      int const nnzBlock = nz_blockJtJ.size();

      vector<int> permBlockJtJ(_totalParamCount + 1);

      if (nnzBlock > 0)
      {
         CCS_Matrix<int> blockJtJ(_totalParamCount, _totalParamCount, nz_blockJtJ);

         int * colStarts = (int *)blockJtJ.getColumnStarts();
         int * rowIdxs   = (int *)blockJtJ.getRowIndices();

         int stats[COLAMD_STATS];
         symamd(_totalParamCount, rowIdxs, colStarts, &permBlockJtJ[0], (double *) NULL, stats, &calloc, &free);
         if (optimizerVerbosenessLevel >= 2) symamd_report(stats);
      }
      else
         for (size_t k = 0; k < permBlockJtJ.size(); ++k) permBlockJtJ[k] = k;

      //cout << "permBlockJtJ = "; displayVector(permBlockJtJ);

      // The number of columns/rows of the scalar JtJ
      int JtJ_size = 0;
      for (int t = 0; t < _paramDesc.nParamTypes; ++t)
      {
         _paramTypeRowStart[t] = JtJ_size;
         JtJ_size += _paramDesc.dimension[t]*_paramDesc.count[t];
      }
      _paramTypeRowStart[_paramDesc.nParamTypes] = JtJ_size;

      // From the determined symbolic permutation with logical (block) variables, determine the actual ordering for all the scalar unknowns
      _perm_JtJ.resize(JtJ_size + 1);

      int curDstCol = 0;
      for (size_t k = 0; k < permBlockJtJ.size()-1; ++k)
      {
         int const srcCol = permBlockJtJ[k];

         int const paramType = _paramIdInverseMap[srcCol].first;
         int const paramIx   = _paramIdInverseMap[srcCol].second;
         int const dimension = _paramDesc.dimension[paramType];

         int const colStart = _paramTypeRowStart[paramType] + paramIx*dimension;

         for (int n = 0; n < dimension; ++n)
            _perm_JtJ[curDstCol + n] = colStart + n;

         curDstCol += dimension;
      } // end for (k)
      _perm_JtJ.back() = _perm_JtJ.size() - 1;
      //cout << "_perm_JtJ = "; displayVector(_perm_JtJ);

      // Finally, compute the inverse of the full permutation.
      _invPerm_JtJ.resize(_perm_JtJ.size());
      for (size_t k = 0; k < _perm_JtJ.size(); ++k) _invPerm_JtJ[_perm_JtJ[k]] = k;
      //cout << "_invPerm_JtJ = "; displayVector(_invPerm_JtJ);

      // Now determine all non-zeros of JtJ (the upper triangular part)
      vector<pair<int, int> > nz_JtJ;
      for (int t1 = 0; t1 < _paramDesc.nParamTypes; ++t1)
         for (int t2 = 0; t2 < _paramDesc.nParamTypes; ++t2)
         {
            if (!_hessian.Hs[t1][t2]) continue;

            vector<pair<int, int> > const& nz = _hessian.nonzeroPairs[t1][t2];
            for (int k = 0; k < nz.size(); ++k)
            {
               int const ix1 = nz[k].first;
               int const ix2 = nz[k].second;

               int const dim1 = _paramDesc.dimension[t1];
               int const dim2 = _paramDesc.dimension[t2];

               int const r0 = _paramTypeRowStart[t1] + ix1 * dim1;
               int const c0 = _paramTypeRowStart[t2] + ix2 * dim2;

               for (int r = 0; r < dim1; ++r)
                  for (int c = 0; c < dim2; ++c)
                  {
#if defined(ONLY_UPPER_TRIANGULAR_HESSIAN)
                     if (r0 + r <= c0 + c) nz_JtJ.push_back(make_pair(r0 + r, c0 + c));
#else
                     nz_JtJ.push_back(make_pair(r0 + r, c0 + c));
#endif
                  }
            } // end for (t1)
         } // end for (t2)
      //cout << "nz_JtJ.size() = " << nz_JtJ.size() << endl;

      // Reorder columns
      for (size_t k = 0; k < nz_JtJ.size(); ++k)
      {
         int const i = nz_JtJ[k].first;
         int const j = nz_JtJ[k].second;

         int pi = _invPerm_JtJ[i];
         int pj = _invPerm_JtJ[j];
#if defined(ONLY_UPPER_TRIANGULAR_HESSIAN)
         // Swap values if in lower triangular part
         if (pi > pj) std::swap(pi, pj);
#endif
         nz_JtJ[k].first = pi;
         nz_JtJ[k].second = pj;
         //cout << "(" << i << ", " << j << ") -> (" << pi << ", " << pj << ")" << endl;
      }

      _JtJ.create(JtJ_size, JtJ_size, nz_JtJ);

      vector<int> workFlags(JtJ_size);

      _JtJ_Lp.resize(JtJ_size+1);
      _JtJ_Parent.resize(JtJ_size);
      _JtJ_Lnz.resize(JtJ_size);

      LDL_symbolic(JtJ_size, (int *)_JtJ.getColumnStarts(), (int *)_JtJ.getRowIndices(),
                   &_JtJ_Lp[0], &_JtJ_Parent[0], &_JtJ_Lnz[0], &workFlags[0]);

      if (optimizerVerbosenessLevel >= 1)
         cout << "NLSQ_LM_Optimizer: Nonzeros in LDL decomposition: " << _JtJ_Lp[JtJ_size] << endl;
   } // end NLSQ_LM_Optimizer::setupSparseJtJ()

   void
   NLSQ_LM_Optimizer::fillJtJ()
   {
      // The following has to replicate the procedure as in serializeNonZerosJtJ()
      int serial = 0;

      double * values = _JtJ.getValues();
      int const * destIdxs = _JtJ.getDestIndices();

      for (int t1 = 0; t1 < _paramDesc.nParamTypes; ++t1)
         for (int t2 = 0; t2 < _paramDesc.nParamTypes; ++t2)
         {
            if (!_hessian.Hs[t1][t2]) continue;

            vector<pair<int, int> > const& nz = _hessian.nonzeroPairs[t1][t2];
            MatrixArray<double>     const& Hs = *_hessian.Hs[t1][t2];

            for (int k = 0; k < nz.size(); ++k)
            {
               int const ix1 = nz[k].first;
               int const ix2 = nz[k].second;

               int const dim1 = _paramDesc.dimension[t1];
               int const dim2 = _paramDesc.dimension[t2];

               int const r0 = _paramTypeRowStart[t1] + ix1 * dim1;
               int const c0 = _paramTypeRowStart[t2] + ix2 * dim2;

               for (int r = 0; r < dim1; ++r)
                  for (int c = 0; c < dim2; ++c)
                  {
#if defined(ONLY_UPPER_TRIANGULAR_HESSIAN)
                     if (r0 + r <= c0 + c)
                     {
                        values[destIdxs[serial]] = Hs[k][r][c];
                        ++serial;
                     }
#else
                     values[destIdxs[serial]] = Hs[k][r][c];
                     ++serial;
#endif
                  } // end for (c)
            } // end for (t1)
         } // end for (t2)

      //cout << "serial = " << serial << endl;
      //displaySparseMatrix(_JtJ);
   } // end NLSQ_LM_Optimizer::fillJtJ()

   void
   NLSQ_LM_Optimizer::fillJacobians()
   {
      int const nObjs = _costFunctions.size();
      for (int obj = 0; obj < nObjs; ++obj)
      {
         NLSQ_CostFunction& costFun = *_costFunctions[obj];
         NLSQ_Residuals& residuals = *_residuals[obj];
         costFun.initializeJacobian();
         costFun.fillAllJacobians(residuals._weights, residuals._Js);
      } // end for (obj)
   } // end NLSQ_LM_Optimizer::fillJacobians()

   void
   NLSQ_LM_Optimizer::fillHessian()
   {
      // Set Hessian to zero
      _hessian.setZero();

      int const nObjs = _costFunctions.size();
      for (int obj = 0; obj < nObjs; ++obj)
      {
         NLSQ_CostFunction& costFun = *_costFunctions[obj];

         int const nParamTypes = costFun._usedParamTypes.size();
         int const nMeasurements = costFun._nMeasurements;

         MatrixArray<int> const& index = *_hessianIndices[obj];
         NLSQ_Residuals   const& residuals = *_residuals[obj];

         vector<int> const& usedParamTypes = costFun._usedParamTypes;

         for (int i1 = 0; i1 < usedParamTypes.size(); ++i1)
         {
            int const t1   = usedParamTypes[i1];
            int const dim1 = _paramDesc.dimension[t1];

            MatrixArray<double> const& Js1 = *residuals._Js[i1];

            for (int i2 = 0; i2 < usedParamTypes.size(); ++i2)
            {
               int const t2   = usedParamTypes[i2];
               int const dim2 = _paramDesc.dimension[t2];

               MatrixArray<double> const& Js2 = *residuals._Js[i2];

               Matrix<double> J1tJ2(dim1, dim2);

               // Ignore non-existent Hessians lying in the lower triangular part.
               if (!_hessian.Hs[t1][t2]) continue;

               MatrixArray<double>& Hs = *_hessian.Hs[t1][t2];

               for (int k = 0; k < nMeasurements; ++k)
               {
                  int const ix1 = costFun._correspondingParams[k][i1];
                  int const id1 = this->getParamId(t1, ix1);
                  int const ix2 = costFun._correspondingParams[k][i2];
                  int const id2 = this->getParamId(t2, ix2);

#if defined(ONLY_UPPER_TRIANGULAR_HESSIAN)
                  if (id1 > id2) continue; // only store the upper diagonal blocks
#endif

                  int const n = index[k][i1][i2];
                  // cout << "obj = " << obj << " i1 = " << i1 << " i2 = " << i2  << " t1 = " << t1 << " t2 = " << t2;
                  // cout << " n = " << n << " Hs.count = " << Hs.count() << endl;
                  assert(n < Hs.count());

                  multiply_At_B(Js1[k], Js2[k], J1tJ2);
                  addMatricesIP(J1tJ2, Hs[n]);
               } // end for (k)
            } // end for (i2)
         } // end for (i1)
      } // end for (obj)
   } // end NLSQ_LM_Optimizer::fillHessian()

   void
   NLSQ_LM_Optimizer::evalJt_e(Vector<double>& Jt_e)
   {
      makeZeroVector(Jt_e);

      int const nObjs = _costFunctions.size();
      for (int obj = 0; obj < nObjs; ++obj)
      {
         NLSQ_CostFunction& costFun = *_costFunctions[obj];
         NLSQ_Residuals& residuals = *_residuals[obj];

         int const nParamTypes   = costFun._usedParamTypes.size();
         int const nMeasurements = costFun._nMeasurements;

         for (int i = 0; i < nParamTypes; ++i)
         {
            int const paramType = costFun._usedParamTypes[i];
            int const paramDim = _paramDesc.dimension[paramType];

            MatrixArray<double> const& J = *residuals._Js[i];

            Vector<double> Jkt_e(paramDim);

            for (int k = 0; k < nMeasurements; ++k)
            {
               int const id = costFun._correspondingParams[k][i];
               int const dstRow = _paramTypeRowStart[paramType] + id*paramDim;;

               multiply_At_v(J[k], residuals._residuals[k], Jkt_e);
               for (int l = 0; l < paramDim; ++l) Jt_e[dstRow + l] += Jkt_e[l];
            } // end for (k)
         } // end for (i)
      } // end for (obj)
   } // end NLSQ_LM_Optimizer::evalJt_e()

   void
   NLSQ_LM_Optimizer::minimize()
   {
      status = LEVENBERG_OPTIMIZER_TIMEOUT;
      bool computeDerivatives = true;

      if (_totalParamCount == 0)
      {
         // No degrees of freedom, nothing to optimize.
         status = LEVENBERG_OPTIMIZER_CONVERGED;
         return;
      }

      int const totalParamDimension = _JtJ.num_cols();

      Vector<double> Jt_e(totalParamDimension);
      Vector<double> delta(totalParamDimension);
      Vector<double> deltaPerm(totalParamDimension);

      double err = 0.0;

      int const nObjs = _costFunctions.size();

      for (currentIteration = 0; currentIteration < maxIterations; ++currentIteration)
      {
         if (optimizerVerbosenessLevel >= 2)
            cout << "ExtSparseLevenbergOptimizer: currentIteration: " << currentIteration << endl;
         if (computeDerivatives)
         {
            vector<double> errors(nObjs);
            err = 0.0;
            for (int obj = 0; obj < nObjs; ++obj)
            {
               NLSQ_CostFunction& costFun = *_costFunctions[obj];
               NLSQ_Residuals& residuals = *_residuals[obj];

               costFun.evalAllResiduals(residuals._residuals);
               costFun.fillAllWeights(residuals._residuals, residuals._weights);

               for (int k = 0; k < costFun._nMeasurements; ++k)
                  scaleVectorIP(-residuals._weights[k], residuals._residuals[k]);

               errors[obj] = squaredResidual(residuals._residuals);
               err += errors[obj];
            } // end for (obj)

            if (optimizerVerbosenessLevel >= 1) cout << "NLSQ_LM_Optimizer: |residual|^2 = " << err << endl;
            if (optimizerVerbosenessLevel >= 2) cout << "NLSQ_LM_Optimizer: residuals (itemized) = "; displayVector(errors);
            if (optimizerVerbosenessLevel >= 2) cout << "NLSQ_LM_Optimizer: lambda = " << lambda << endl;

            this->fillJacobians();
            this->evalJt_e(Jt_e);

            if (this->applyGradientStoppingCriteria(norm_Linf(Jt_e)))
            {
               status = LEVENBERG_OPTIMIZER_CONVERGED;
               goto end;
            }

            this->fillHessian();

            if (currentIteration == 0)
            {
               // Initialize lambda as tau*max(JtJ[i][i])
               double maxEl = -1e30;
               if (optimizerVerbosenessLevel < 2)
               {
                  for (int paramType = 0; paramType < _paramDesc.nParamTypes; ++paramType)
                  {
                     MatrixArray<double> const& Hs = *_hessian.Hs[paramType][paramType];
                     int const dim = Hs.num_cols();
                     int const count = Hs.count();
                     for (int n = 0; n < count; ++n)
                        for (int l = 0; l < dim; ++l) maxEl = std::max(maxEl, Hs[n][l][l]);
                  } // end for (paramType)
               }
               else
               {
                  int maxParamType = -1;
                  int maxElIndex = -1;
                  int maxElRow = -1;

                  for (int paramType = 0; paramType < _paramDesc.nParamTypes; ++paramType)
                  {
                     MatrixArray<double> const& Hs = *_hessian.Hs[paramType][paramType];
                     int const dim = Hs.num_cols();
                     int const count = Hs.count();
                     for (int n = 0; n < count; ++n)
                        for (int l = 0; l < dim; ++l)
                        {
                           double const el = Hs[n][l][l];
                           if (el > maxEl)
                           {
                              maxEl = el;
                              maxParamType = paramType;
                              maxElIndex   = n;
                              maxElRow     = l;
                           } // end if
                        } // end for (l)
                  } // end for (paramType)

                  cout << "NLSQ_LM_Optimizer: initial lambda = " << tau*maxEl << "; max diagonal element found at "
                       << maxParamType << "[" << maxElIndex << "][" << maxElRow << "]" << endl;
               } // end if

               lambda = tau * maxEl;
            } // end if (currentIteration == 0)
         } // end if (computeDerivatives)

         // Augment the diagonals
         for (int paramType = 0; paramType < _paramDesc.nParamTypes; ++paramType)
         {
            MatrixArray<double>& Hs = *_hessian.Hs[paramType][paramType];
            vector<pair<int, int> > const& nzPairs = _hessian.nonzeroPairs[paramType][paramType];
            int const dim = Hs.num_cols();
            int const count = Hs.count();

            // Only augment those with the same parameter id
            for (int n = 0; n < count; ++n)
            {
               if (nzPairs[n].first != nzPairs[n].second) continue;
               for (int l = 0; l < dim; ++l)
                  Hs[n][l][l] += lambda;
            }
         } // end for (paramType)

         this->fillJtJ();

         bool success = true;
         double rho = 0.0;
         {
            int const nCols = _JtJ_Parent.size();
            //int const nnz   = _JtJ.getNonzeroCount();
            int const lnz   = _JtJ_Lp.back();

            vector<int> Li(lnz);
            vector<double> Lx(lnz);
            vector<double> D(nCols), Y(nCols);
            vector<int> workPattern(nCols), workFlag(nCols);

            int * colStarts = (int *)_JtJ.getColumnStarts();
            int * rowIdxs   = (int *)_JtJ.getRowIndices();
            double * values = _JtJ.getValues();

            int const d = LDL_numeric(nCols, colStarts, rowIdxs, values,
                                      &_JtJ_Lp[0], &_JtJ_Parent[0], &_JtJ_Lnz[0],
                                      &Li[0], &Lx[0], &D[0],
                                      &Y[0], &workPattern[0], &workFlag[0]);

            if (d == nCols)
            {
               LDL_perm(nCols, &deltaPerm[0], &Jt_e[0], &_perm_JtJ[0]);
               LDL_lsolve(nCols, &deltaPerm[0], &_JtJ_Lp[0], &Li[0], &Lx[0]);
               LDL_dsolve(nCols, &deltaPerm[0], &D[0]);
               LDL_ltsolve(nCols, &deltaPerm[0], &_JtJ_Lp[0], &Li[0], &Lx[0]);
               LDL_permt(nCols, &delta[0], &deltaPerm[0], &_perm_JtJ[0]);
            }
            else
            {
               if (optimizerVerbosenessLevel >= 2)
                  cout << "NLSQ_LM_Optimizer: LDL decomposition failed with d = " << d << ". Increasing lambda." << endl;
               success = false;
            }
         }

         double deltaError = 0;

         if (success)
         {
            double const deltaSqrLength = sqrNorm_L2(delta);
            double const paramLength = this->getParameterLength();

            if (optimizerVerbosenessLevel >= 2)
               cout << "NLSQ_LM_Optimizer: ||delta|| = " << sqrt(deltaSqrLength) << " ||paramLength|| = " << paramLength << endl;

            if (this->applyUpdateStoppingCriteria(paramLength, sqrt(deltaSqrLength)))
            {
               status = LEVENBERG_OPTIMIZER_SMALL_UPDATE;
               goto end;
            }

            saveAllParameters();

            for (int paramType = 0; paramType < _paramDesc.nParamTypes; ++paramType)
            {
               int const paramDim = _paramDesc.dimension[paramType];
               int const count    = _paramDesc.count[paramType];
               int const rowStart = _paramTypeRowStart[paramType];

               VectorArrayAdapter<double> deltaParam(count, paramDim, &delta[0] + rowStart);
               this->updateParameters(paramType, deltaParam);
            } // end for (paramType)

            double newErr = 0.0;
            vector<double> errors(nObjs);
            for (int obj = 0; obj < nObjs; ++obj)
            {
               NLSQ_CostFunction& costFun = *_costFunctions[obj];
               NLSQ_Residuals& residuals = *_residuals[obj];

               costFun.evalAllResiduals(residuals._residuals);
               costFun.fillAllWeights(residuals._residuals, residuals._weights);
               for (int k = 0; k < costFun._nMeasurements; ++k)
                  scaleVectorIP(residuals._weights[k], residuals._residuals[k]);
               errors[obj] = squaredResidual(residuals._residuals);
               newErr += errors[obj];
            } // end for (obj)

            deltaError = newErr - err;

            rho = err - newErr;
            if (optimizerVerbosenessLevel >= 2)
            {
               cout << "NLSQ_LM_Optimizer: |new residual|^2 = " << newErr << " rho = " << rho << endl;
               cout << "NLSQ_LM_Optimizer: new residuals (itemized) = "; displayVector(errors);
            }

            double const denom1 = lambda * deltaSqrLength;
            double const denom2 = innerProduct(delta, Jt_e);
            rho = rho / (denom1 + denom2);
            if (optimizerVerbosenessLevel >= 2)
               cout << "NLSQ_LM_Optimizer: rho = " << rho
                    << " denom1 = " << denom1 << " denom2 = " << denom2 << endl;
         } // end if (success)

         if (success && deltaError < 0)
         {
            if (this->applyImprovementStoppingCriteria(deltaError, err))
            {
               if (optimizerVerbosenessLevel >= 2)
                  cout << "NLSQ_LM_Optimizer: too small improvement in cost function, exiting." << endl;
               goto end;
            }
         }

         if (success && rho > 0)
         {
            if (optimizerVerbosenessLevel >= 2)
               cout << "NLSQ_LM_Optimizer: Improved solution - decreasing lambda." << endl;
            // Improvement in the new solution
            decreaseLambda(rho);
            if (optimizerVerbosenessLevel >= 2) cout << "NLSQ_LM_Optimizer: new lambda = " << lambda << endl;
            computeDerivatives = true;
         }
         else
         {
            if (optimizerVerbosenessLevel >= 2)
               cout << "NLSQ_LM_Optimizer: Inferior solution - increasing lambda." << endl;
            restoreAllParameters();

            // Undo the augmentation of the diagonals
            for (int paramType = 0; paramType < _paramDesc.nParamTypes; ++paramType)
            {
               MatrixArray<double>& Hs = *_hessian.Hs[paramType][paramType];
               int const dim = Hs.num_cols();
               int const count = Hs.count();
               for (int n = 0; n < count; ++n)
                  for (int l = 0; l < dim; ++l)
                     Hs[n][l][l] -= lambda;
            } // end for (paramType)

            increaseLambda();
            if (optimizerVerbosenessLevel >= 2) cout << "NLSQ_LM_Optimizer: new lambda = " << lambda << endl;
            computeDerivatives = false;
         } // end if
      } // end for (currentIteration)

     end:;
      if (optimizerVerbosenessLevel >= 2)
         cout << "Leaving NLSQ_LM_Optimizer::minimize()." << endl;
   } // end NLSQ_LM_Optimizer::minimize()

} // end namespace V3D

#endif
