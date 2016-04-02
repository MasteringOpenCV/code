#include "Math/v3d_optimization.h"

#if defined(V3DLIB_ENABLE_SUITESPARSE)
//# include "COLAMD/Include/colamd.h"
# include "colamd.h"
# if 0
extern "C"
{
//#  include "LDL/Include/ldl.h"
#  include "ldl.h"
}
# else
#  include "Math/v3d_ldl_private.h"
# endif
#endif

#include <iostream>
#include <map>

//#define USE_MULTIPLICATIVE_UPDATE 1

using namespace std;


namespace V3D
{

#if defined(V3DLIB_ENABLE_SUITESPARSE)

   void
   SparseLM_CostFunction::fillAllJacobians()
   {
      int const nVaryingA = _paramInfo.nVaryingA;
      int const nVaryingB = _paramInfo.nVaryingB;
      int const nVaryingC = _paramInfo.nVaryingC;

      for (int k = 0; k < _nMeasurements; ++k)
      {
         int const i = _correspondingParamA[k];
         int const j = _correspondingParamB[k];

         if (i < _paramInfo.nNonvaryingA && j < _paramInfo.nNonvaryingB) continue;

         fillJacobians(_Ak[k], _Bk[k], _Ck[k], i, j, k);

         // Clear the Jacobians for locked parameters.
         if (i < _paramInfo.nNonvaryingA) makeZeroMatrix(_Ak[k]);
         if (j < _paramInfo.nNonvaryingB) makeZeroMatrix(_Bk[k]);

         if (_paramInfo.nNonvaryingC > 0)
         {
            for (int r = 0; r < _measurementDimension; ++r)
               for (int c = 0; c < _paramInfo.nNonvaryingC; ++c) _Ck[k][r][c] = 0.0;
         }
      } // end for (k)

      if (nVaryingA > 0)
      {
         for (int k = 0; k < _nMeasurements; ++k)
            scaleMatrixIP(_weights[k], _Ak[k]);
      }
      if (nVaryingB > 0)
      {
         for (int k = 0; k < _nMeasurements; ++k)
            scaleMatrixIP(_weights[k], _Bk[k]);
      }
      if (nVaryingC > 0)
      {
         for (int k = 0; k < _nMeasurements; ++k)
            scaleMatrixIP(_weights[k], _Ck[k]);
      }
   } // end SparseLM_CostFunction::fillAllJacobians()

   void
   SparseLM_CostFunction::fillHessian(std::vector<int> const& jointIndexW,
                                      MatrixArray<double>& Ui, MatrixArray<double>& Vj,
                                      MatrixArray<double>& Wn,
                                      Matrix<double>& X, Matrix<double>& Y, Matrix<double>& Z)
   {
      // The lhs J^t*J consists of several parts:
      //         [ U     W   X ]
      // J^t*J = [ W^t   V   Y ]
      //         [ X^t  Y^t  Z ],
      // where U, V and W are block-sparse matrices (due to the sparsity of A and B).
      // X, Y and Z contain only a few columns (the number of global parameters).

      int const nParametersA = _paramInfo.nParametersA;
      int const nParametersB = _paramInfo.nParametersB;
      int const nNonvaryingA = _paramInfo.nNonvaryingA;
      int const nNonvaryingB = _paramInfo.nNonvaryingB;
      int const nNonvaryingC = _paramInfo.nNonvaryingC;
      int const paramDimensionA = _paramInfo.paramDimensionA;
      int const paramDimensionB = _paramInfo.paramDimensionB;
      int const paramDimensionC = _paramInfo.paramDimensionC;
      int const nVaryingA = nParametersA - nNonvaryingA;
      int const nVaryingB = nParametersB - nNonvaryingB;
      int const nVaryingC = paramDimensionC - nNonvaryingC;

      if (nVaryingA > 0)
      {
         // Compute Ui
         Matrix<double> U(paramDimensionA, paramDimensionA);

         for (int k = 0; k < _nMeasurements; ++k)
         {
            int const i = _correspondingParamA[k] - nNonvaryingA;
            if (i < 0) continue;
            multiply_At_A(_Ak[k], U);
            addMatricesIP(U, Ui[i]);
         } // end for (k)
      } // end if

      if (nVaryingB > 0)
      {
         // Compute Vj
         Matrix<double> V(paramDimensionB, paramDimensionB);

         for (int k = 0; k < _nMeasurements; ++k)
         {
            int const j = _correspondingParamB[k] - nNonvaryingB;
            if (j < 0) continue;
            multiply_At_A(_Bk[k], V);
            addMatricesIP(V, Vj[j]);
         } // end for (k)
      } // end if

      if (nVaryingC > 0)
      {
         Matrix<double> ZZ(paramDimensionC, paramDimensionC);
         Matrix<double> Zsum(paramDimensionC, paramDimensionC);

         makeZeroMatrix(Zsum);

         for (int k = 0; k < _nMeasurements; ++k)
         {
            multiply_At_A(_Ck[k], ZZ);
            addMatricesIP(ZZ, Zsum);
         } // end for (k)

         // Ignore the non-varying parameters
         for (int i = 0; i < nVaryingC; ++i)
            for (int j = 0; j < nVaryingC; ++j)
               Z[i][j] += Zsum[i+nNonvaryingC][j+nNonvaryingC];
      } // end if

      if (nVaryingA > 0 && nVaryingB > 0)
      {
         Matrix<double> W(paramDimensionA, paramDimensionB);

         for (int k = 0; k < _nMeasurements; ++k)
         {
            int const n = jointIndexW[k];
            if (n >= 0)
            {
               multiply_At_B(_Ak[k], _Bk[k], W);
               addMatricesIP(W, Wn[n]);
            } // end if
         } // end for (k)
      } // end if

      if (nVaryingA > 0 && nVaryingC > 0)
      {
         Matrix<double> XX(paramDimensionA, paramDimensionC);

         for (int k = 0; k < _nMeasurements; ++k)
         {
            int const i = _correspondingParamA[k] - nNonvaryingA;
            // Ignore the non-varying parameters
            if (i < 0) continue;

            multiply_At_B(_Ak[k], _Ck[k], XX);

            for (int r = 0; r < paramDimensionA; ++r)
               for (int c = 0; c < nVaryingC; ++c)
                  X[r+i*paramDimensionA][c] += XX[r][c+nNonvaryingC];
         } // end for (k)
      } // end if

      if (nVaryingB > 0 && nVaryingC > 0)
      {
         Matrix<double> YY(paramDimensionB, paramDimensionC);

         for (int k = 0; k < _nMeasurements; ++k)
         {
            int const j = _correspondingParamB[k] - nNonvaryingB;
            // Ignore the non-varying parameters
            if (j < 0) continue;

            multiply_At_B(_Bk[k], _Ck[k], YY);

            for (int r = 0; r < paramDimensionB; ++r)
               for (int c = 0; c < nVaryingC; ++c)
                  Y[r+j*paramDimensionB][c] += YY[r][c+nNonvaryingC];
         } // end for (k)
      } // end if
   } // end SparseLM_CostFunction::fillHessian()

   void
   SparseLM_CostFunction::evalJt_e(VectorArray<double>& At_e, VectorArray<double>& Bt_e, Vector<double>& Ct_e)
   {
      int const nParametersA = _paramInfo.nParametersA;
      int const nParametersB = _paramInfo.nParametersB;
      int const nNonvaryingA = _paramInfo.nNonvaryingA;
      int const nNonvaryingB = _paramInfo.nNonvaryingB;
      int const nNonvaryingC = _paramInfo.nNonvaryingC;
      int const paramDimensionA = _paramInfo.paramDimensionA;
      int const paramDimensionB = _paramInfo.paramDimensionB;
      int const paramDimensionC = _paramInfo.paramDimensionC;
      int const nVaryingA = nParametersA - nNonvaryingA;
      int const nVaryingB = nParametersB - nNonvaryingB;
      int const nVaryingC = paramDimensionC - nNonvaryingC;

      // Compute the different parts of J^t*e
      if (nVaryingA > 0)
      {
         Vector<double> tmp(paramDimensionA);

         for (int k = 0; k < _nMeasurements; ++k)
         {
            int const i = _correspondingParamA[k] - nNonvaryingA;
            if (i < 0) continue;
            multiply_At_v(_Ak[k], _residuals[k], tmp);
            addVectors(tmp, At_e[i], At_e[i]);
         } // end for (k)
      } // end if

      if (nVaryingB > 0)
      {
         Vector<double> tmp(paramDimensionB);

         for (int k = 0; k < _nMeasurements; ++k)
         {
            int const j = _correspondingParamB[k] - nNonvaryingB;
            if (j < 0) continue;
            multiply_At_v(_Bk[k], _residuals[k], tmp);
            addVectors(tmp, Bt_e[j], Bt_e[j]);
         } // end for (k)
      } // end if

      if (nVaryingC > 0)
      {
         Vector<double> tmp(paramDimensionC);

         for (int k = 0; k < _nMeasurements; ++k)
         {
            multiply_At_v(_Ck[k], _residuals[k], tmp);
            for (int l = 0; l < nVaryingC; ++l) Ct_e[l] += tmp[nNonvaryingC + l];
         }
      } // end if
   } // end SparseLM_CostFunction::evalJt_e()

//======================================================================

   void
   ExtSparseLevenbergOptimizer::setupSparseJtJ()
   {
      int const nParametersA = _paramInfo.nParametersA;
      int const nParametersB = _paramInfo.nParametersB;
      int const nNonvaryingA = _paramInfo.nNonvaryingA;
      int const nNonvaryingB = _paramInfo.nNonvaryingB;
      int const nNonvaryingC = _paramInfo.nNonvaryingC;
      int const paramDimensionA = _paramInfo.paramDimensionA;
      int const paramDimensionB = _paramInfo.paramDimensionB;
      int const paramDimensionC = _paramInfo.paramDimensionC;
      int const nVaryingA = nParametersA - nNonvaryingA;
      int const nVaryingB = nParametersB - nNonvaryingB;
      int const nVaryingC = paramDimensionC - nNonvaryingC;

      int const bColumnStart = nVaryingA*paramDimensionA;
      int const cColumnStart = bColumnStart + nVaryingB*paramDimensionB;
      int const nColumns     = cColumnStart + nVaryingC;

      int const nObjs = _costFunctions.size();

      _jointIndexW.resize(nObjs);
      _jointNonzerosW.clear();

      map<pair<int, int>, int> jointNonzeroMap;

      for (int obj = 0; obj < nObjs; ++obj)
      {
         SparseLM_CostFunction& costFun = *_costFunctions[obj];
         vector<int>& jointIndexW = _jointIndexW[obj];
         jointIndexW.resize(costFun._nMeasurements);

         for (int k = 0; k < costFun._nMeasurements; ++k)
         {
            int const i = costFun._correspondingParamA[k] - nNonvaryingA;
            int const j = costFun._correspondingParamB[k] - nNonvaryingB;
            if (i >= 0 && j >= 0)
            {
               map<pair<int, int>, int>::const_iterator p = jointNonzeroMap.find(make_pair(i, j));
               if (p == jointNonzeroMap.end())
               {
                  jointNonzeroMap.insert(make_pair(make_pair(i, j), _jointNonzerosW.size()));
                  jointIndexW[k] = _jointNonzerosW.size();
                  _jointNonzerosW.push_back(make_pair(i, j));
               }
               else
                  jointIndexW[k] = (*p).second;
            } // end if
         } // end for (k)
      } // end for (obj)

      int const bBlockColumnStart = nVaryingA;
      int const cBlockColumnStart = bBlockColumnStart + nVaryingB;

      // If there are C parameters to optimize over, this counts as one column in the block representation
      int const nBlockColumns = cBlockColumnStart + ((nVaryingC > 0) ? 1 : 0);
      //cout << "nBlockColumns = " << nBlockColumns << endl;

      // For the column reordering we treat the columns belonging to one set
      // of parameters as one (logical) column.

      // Determine non-zeros of JtJ (we forget about the non-zero diagonal for now)
      // Only consider nonzeros of Ai^t * Bj induced by the measurements.
      vector<pair<int, int> > nz_blockJtJ(_jointNonzerosW.size());
      for (size_t k = 0; k < _jointNonzerosW.size(); ++k)
      {
         nz_blockJtJ[k].first  = _jointNonzerosW[k].second + bBlockColumnStart;
         nz_blockJtJ[k].second = _jointNonzerosW[k].first;
      }

      if (nVaryingC > 0)
      {
         // We assume, that the global unknowns are linked to every other variable.
         for (int i = 0; i < nVaryingA; ++i)
            nz_blockJtJ.push_back(make_pair(cBlockColumnStart, i));
         for (int j = 0; j < nVaryingB; ++j)
            nz_blockJtJ.push_back(make_pair(cBlockColumnStart, j + bBlockColumnStart));
      } // end if

      int const nnzBlock = nz_blockJtJ.size();

      vector<int> permBlockJtJ(nBlockColumns + 1);

      if (nnzBlock > 0)
      {
//          cout << "nnzBlock = " << nnzBlock << endl;

         CCS_Matrix<int> blockJtJ(nBlockColumns, nBlockColumns, nz_blockJtJ);

//          cout << " nz_blockJtJ: " << endl;
//          for (size_t k = 0; k < nz_blockJtJ.size(); ++k)
//             cout << " " << nz_blockJtJ[k].first << ":" << nz_blockJtJ[k].second << endl;
//          cout << endl;

         int * colStarts = (int *)blockJtJ.getColumnStarts();
         int * rowIdxs   = (int *)blockJtJ.getRowIndices();

//          cout << "blockJtJ_colStarts = ";
//          for (int k = 0; k <= nBlockColumns; ++k) cout << colStarts[k] << " ";
//          cout << endl;

//          cout << "blockJtJ_rowIdxs = ";
//          for (int k = 0; k < nnzBlock; ++k) cout << rowIdxs[k] << " ";
//          cout << endl;

         int stats[COLAMD_STATS];
         symamd(nBlockColumns, rowIdxs, colStarts, &permBlockJtJ[0], (double *) NULL, stats, &calloc, &free);
         if (optimizerVerbosenessLevel >= 2) symamd_report(stats);
      }
      else
         for (size_t k = 0; k < permBlockJtJ.size(); ++k) permBlockJtJ[k] = k;

//       cout << "permBlockJtJ = ";
//       for (int k = 0; k < permBlockJtJ.size(); ++k)
//          cout << permBlockJtJ[k] << " ";
//       cout << endl;

      // From the determined symbolic permutation with logical variables, determine the actual ordering
      _perm_JtJ.resize(nVaryingA*paramDimensionA + nVaryingB*paramDimensionB + nVaryingC + 1);

      int curDstCol = 0;
      for (size_t k = 0; k < permBlockJtJ.size()-1; ++k)
      {
         int const srcCol = permBlockJtJ[k];
         if (srcCol < nVaryingA)
         {
            for (int n = 0; n < paramDimensionA; ++n)
               _perm_JtJ[curDstCol + n] = srcCol*paramDimensionA + n;
            curDstCol += paramDimensionA;
         }
         else if (srcCol >= bBlockColumnStart && srcCol < cBlockColumnStart)
         {
            int const bStart = nVaryingA*paramDimensionA;
            int const j = srcCol - bBlockColumnStart;

            for (int n = 0; n < paramDimensionB; ++n)
               _perm_JtJ[curDstCol + n] = bStart + j*paramDimensionB + n;
            curDstCol += paramDimensionB;
         }
         else if (srcCol == cBlockColumnStart)
         {
            int const cStart = nVaryingA*paramDimensionA + nVaryingB*paramDimensionB;

            for (int n = 0; n < nVaryingC; ++n)
               _perm_JtJ[curDstCol + n] = cStart + n;
            curDstCol += nVaryingC;
         }
         else
            throwV3DErrorHere("setupJtJ(): column out of bounds");
      } // end for (k)
      _perm_JtJ.back() = _perm_JtJ.size() - 1;

//       cout << "_perm_JtJ = ";
//       for (int k = 0; k < _perm_JtJ.size(); ++k) cout << _perm_JtJ[k] << " ";
//       cout << endl;

      // Finally, compute the inverse of the full permutation.
      _invPerm_JtJ.resize(_perm_JtJ.size());
      for (size_t k = 0; k < _perm_JtJ.size(); ++k) _invPerm_JtJ[_perm_JtJ[k]] = k;

      vector<pair<int, int> > nz_JtJ;
      this->serializeNonZerosJtJ(nz_JtJ);

      for (size_t k = 0; k < nz_JtJ.size(); ++k)
      {
         int const i = nz_JtJ[k].first;
         int const j = nz_JtJ[k].second;

         int pi = _invPerm_JtJ[i];
         int pj = _invPerm_JtJ[j];
         // Swap values if in lower triangular part
         if (pi > pj) std::swap(pi, pj);
         nz_JtJ[k].first = pi;
         nz_JtJ[k].second = pj;
      }

      _JtJ.create(nColumns, nColumns, nz_JtJ);

      vector<int> workFlags(nColumns);

      _JtJ_Lp.resize(nColumns+1);
      _JtJ_Parent.resize(nColumns);
      _JtJ_Lnz.resize(nColumns);

      LDL_symbolic(nColumns, (int *)_JtJ.getColumnStarts(), (int *)_JtJ.getRowIndices(),
                   &_JtJ_Lp[0], &_JtJ_Parent[0], &_JtJ_Lnz[0],
                   &workFlags[0]);

      if (optimizerVerbosenessLevel >= 1)
         cout << "ExtSparseLevenbergOptimizer: Nonzeros in LDL decomposition: " << _JtJ_Lp[nColumns] << endl;
   } // end ExtSparseLevenbergOptimizer::setupSparseJtJ()

   void
   ExtSparseLevenbergOptimizer::serializeNonZerosJtJ(vector<pair<int, int> >& dst) const
   {
      int const nParametersA = _paramInfo.nParametersA;
      int const nParametersB = _paramInfo.nParametersB;
      int const nNonvaryingA = _paramInfo.nNonvaryingA;
      int const nNonvaryingB = _paramInfo.nNonvaryingB;
      int const nNonvaryingC = _paramInfo.nNonvaryingC;
      int const paramDimensionA = _paramInfo.paramDimensionA;
      int const paramDimensionB = _paramInfo.paramDimensionB;
      int const paramDimensionC = _paramInfo.paramDimensionC;
      int const nVaryingA = nParametersA - nNonvaryingA;
      int const nVaryingB = nParametersB - nNonvaryingB;
      int const nVaryingC = paramDimensionC - nNonvaryingC;

      int const bColumnStart = nVaryingA*paramDimensionA;
      int const cColumnStart = bColumnStart + nVaryingB*paramDimensionB;

      dst.clear();

      // Add the diagonal block matrices (only the upper triangular part).

      // Ui submatrices of JtJ
      for (int i = 0; i < nVaryingA; ++i)
      {
         int const i0 = i * paramDimensionA;

         for (int c = 0; c < paramDimensionA; ++c)
            for (int r = 0; r <= c; ++r)
               dst.push_back(make_pair(i0 + r, i0 + c));
      }

      // Vj submatrices of JtJ
      for (int j = 0; j < nVaryingB; ++j)
      {
         int const j0 = j*paramDimensionB + bColumnStart;

         for (int c = 0; c < paramDimensionB; ++c)
            for (int r = 0; r <= c; ++r)
               dst.push_back(make_pair(j0 + r, j0 + c));
      }

      // Z submatrix of JtJ
      for (int c = 0; c < nVaryingC; ++c)
         for (int r = 0; r <= c; ++r)
            dst.push_back(make_pair(cColumnStart + r, cColumnStart + c));

      // Add the elements i and j linked by an observation k
      // W submatrix of JtJ
      for (size_t n = 0; n < _jointNonzerosW.size(); ++n)
      {
         int const i0 = _jointNonzerosW[n].first;
         int const j0 = _jointNonzerosW[n].second;
         int const r0 = i0*paramDimensionA;
         int const c0 = j0*paramDimensionB + bColumnStart;

         for (int r = 0; r < paramDimensionA; ++r)
            for (int c = 0; c < paramDimensionB; ++c)
               dst.push_back(make_pair(r0 + r, c0 + c));
      } // end for (n)

      if (nVaryingC > 0)
      {
         // Finally, add the dense columns linking i (resp. j) with the global parameters.
         // X submatrix of JtJ
         for (int i = 0; i < nVaryingA; ++i)
         {
            int const i0 = i*paramDimensionA;

            for (int r = 0; r < paramDimensionA; ++r)
               for (int c = 0; c < nVaryingC; ++c)
                  dst.push_back(make_pair(i0 + r, cColumnStart + c));
         }

         // Y submatrix of JtJ
         for (int j = 0; j < nVaryingB; ++j)
         {
            int const j0 = j*paramDimensionB + bColumnStart;

            for (int r = 0; r < paramDimensionB; ++r)
               for (int c = 0; c < nVaryingC; ++c)
                  dst.push_back(make_pair(j0 + r, cColumnStart + c));
         }
      } // end if
   } // end ExtSparseLevenbergOptimizer::serializeNonZerosJtJ()

   void
   ExtSparseLevenbergOptimizer::fillJtJ(MatrixArray<double> const& Ui,
                                        MatrixArray<double> const& Vj,
                                        MatrixArray<double> const& Wn,
                                        Matrix<double> const& Z,
                                        Matrix<double> const& X,
                                        Matrix<double> const& Y)
   {
      int const nParametersA = _paramInfo.nParametersA;
      int const nParametersB = _paramInfo.nParametersB;
      int const nNonvaryingA = _paramInfo.nNonvaryingA;
      int const nNonvaryingB = _paramInfo.nNonvaryingB;
      int const nNonvaryingC = _paramInfo.nNonvaryingC;
      int const paramDimensionA = _paramInfo.paramDimensionA;
      int const paramDimensionB = _paramInfo.paramDimensionB;
      int const paramDimensionC = _paramInfo.paramDimensionC;
      int const nVaryingA = nParametersA - nNonvaryingA;
      int const nVaryingB = nParametersB - nNonvaryingB;
      int const nVaryingC = paramDimensionC - nNonvaryingC;

      int const bColumnStart = nVaryingA*paramDimensionA;
      //int const cColumnStart = bColumnStart + nVaryingB*_paramDimensionB;

      //int const nCols = _JtJ.num_cols();
      //int const nnz   = _JtJ.getNonzeroCount();

      // The following has to replicate the procedure as in serializeNonZerosJtJ()

      int serial = 0;

      double * values = _JtJ.getValues();
      int const * destIdxs = _JtJ.getDestIndices();

      // Add the diagonal block matrices (only the upper triangular part).

      // Ui submatrices of JtJ
      for (int i = 0; i < nVaryingA; ++i)
      {
         //int const i0 = i * _paramDimensionA;

         for (int c = 0; c < paramDimensionA; ++c)
            for (int r = 0; r <= c; ++r, ++serial)
               values[destIdxs[serial]] = Ui[i][r][c];
      }

      // Vj submatrices of JtJ
      for (int j = 0; j < nVaryingB; ++j)
      {
         //int const j0 = j*_paramDimensionB + bColumnStart;

         for (int c = 0; c < paramDimensionB; ++c)
            for (int r = 0; r <= c; ++r, ++serial)
               values[destIdxs[serial]] = Vj[j][r][c];
      }

      // Z submatrix of JtJ
      for (int c = 0; c < nVaryingC; ++c)
         for (int r = 0; r <= c; ++r, ++serial)
            values[destIdxs[serial]] = Z[r][c];

      // Add the elements i and j linked by an observation k
      // W submatrix of JtJ
      for (size_t n = 0; n < _jointNonzerosW.size(); ++n)
      {
         for (int r = 0; r < paramDimensionA; ++r)
            for (int c = 0; c < paramDimensionB; ++c, ++serial)
               values[destIdxs[serial]] = Wn[n][r][c];
      } // end for (k)

      if (nVaryingC > 0)
      {
         // Finally, add the dense columns linking i (resp. j) with the global parameters.
         // X submatrix of JtJ
         for (int i = 0; i < nVaryingA; ++i)
         {
            int const r0 = i * paramDimensionA;
            for (int r = 0; r < paramDimensionA; ++r)
               for (int c = 0; c < nVaryingC; ++c, ++serial)
                  values[destIdxs[serial]] = X[r0+r][c];
         }

         // Y submatrix of JtJ
         for (int j = 0; j < nVaryingB; ++j)
         {
            int const r0 = j * paramDimensionB;
            for (int r = 0; r < paramDimensionB; ++r)
               for (int c = 0; c < nVaryingC; ++c, ++serial)
                  values[destIdxs[serial]] = Y[r0+r][c];
         }
      } // end if
   } // end ExtSparseLevenbergOptimizer::fillJtJ()

   void
   ExtSparseLevenbergOptimizer::minimize()
   {
      status = LEVENBERG_OPTIMIZER_TIMEOUT;
      bool computeDerivatives = true;

      int const nParametersA = _paramInfo.nParametersA;
      int const nParametersB = _paramInfo.nParametersB;
      int const nNonvaryingA = _paramInfo.nNonvaryingA;
      int const nNonvaryingB = _paramInfo.nNonvaryingB;
      int const nNonvaryingC = _paramInfo.nNonvaryingC;
      int const paramDimensionA = _paramInfo.paramDimensionA;
      int const paramDimensionB = _paramInfo.paramDimensionB;
      int const paramDimensionC = _paramInfo.paramDimensionC;
      int const nVaryingA = nParametersA - nNonvaryingA;
      int const nVaryingB = nParametersB - nNonvaryingB;
      int const nVaryingC = paramDimensionC - nNonvaryingC;

      if (nVaryingA == 0 && nVaryingB == 0 && nVaryingC == 0)
      {
         // No degrees of freedom, nothing to optimize.
         status = LEVENBERG_OPTIMIZER_CONVERGED;
         return;
      }

      this->setupSparseJtJ();

      // Wn = Ak^t*Bk
      MatrixArray<double> Wn(_jointNonzerosW.size(), paramDimensionA, paramDimensionB);

      //VectorArray<double> residuals2(_nMeasurements, _measurementDimension);

      VectorArray<double> diagUi(nVaryingA, paramDimensionA);
      VectorArray<double> diagVj(nVaryingB, paramDimensionB);
      Vector<double> diagZ(nVaryingC);

      VectorArray<double> At_e(nVaryingA, paramDimensionA);
      VectorArray<double> Bt_e(nVaryingB, paramDimensionB);
      Vector<double> Ct_e(nVaryingC);

      Vector<double> Jt_e(nVaryingA*paramDimensionA + nVaryingB*paramDimensionB + nVaryingC);

      Vector<double> delta(nVaryingA*paramDimensionA + nVaryingB*paramDimensionB + nVaryingC);
      Vector<double> deltaPerm(nVaryingA*paramDimensionA + nVaryingB*paramDimensionB + nVaryingC);

      VectorArray<double> deltaAi(nParametersA, paramDimensionA);
      VectorArray<double> deltaBj(nParametersB, paramDimensionB);
      Vector<double> deltaC(paramDimensionC);

      double err = 0.0;

      int const nObjs = _costFunctions.size();

      for (currentIteration = 0; currentIteration < maxIterations; ++currentIteration)
      {
         if (optimizerVerbosenessLevel >= 2)
            cout << "ExtSparseLevenbergOptimizer: currentIteration: " << currentIteration << endl;
         if (computeDerivatives)
         {
            err = 0.0;
            for (int obj = 0; obj < nObjs; ++obj)
            {
               SparseLM_CostFunction& costFun = *_costFunctions[obj];
               costFun.evalResidual(costFun._residuals);
               costFun.fillWeights(costFun._residuals, costFun._weights);
               for (int k = 0; k < costFun._nMeasurements; ++k)
                  scaleVectorIP(-costFun._weights[k], costFun._residuals[k]);

               err += squaredResidual(costFun._residuals);
            } // end for (obj)

            if (optimizerVerbosenessLevel >= 1) cout << "ExtSparseLevenbergOptimizer: |residual|^2 = " << err << endl;
            if (optimizerVerbosenessLevel >= 2) cout << "ExtSparseLevenbergOptimizer: lambda = " << lambda << endl;

            //for (size_t k = 0; k < residuals.count(); ++k) scaleVectorIP(-1.0, residuals[k]);

            for (int obj = 0; obj < nObjs; ++obj)
            {
               SparseLM_CostFunction& costFun = *_costFunctions[obj];
               costFun.setupJacobianGathering();
               costFun.fillAllJacobians();
            } // end for (obj)

            for (int i = 0; i < nVaryingA; ++i) makeZeroVector(At_e[i]);
            for (int j = 0; j < nVaryingB; ++j) makeZeroVector(Bt_e[j]);
            makeZeroVector(Ct_e);

            for (int obj = 0; obj < nObjs; ++obj)
               _costFunctions[obj]->evalJt_e(At_e, Bt_e, Ct_e);

            int pos = 0;
            for (int i = 0; i < nVaryingA; ++i)
               for (int l = 0; l < paramDimensionA; ++l, ++pos)
                  Jt_e[pos] = At_e[i][l];
            for (int j = 0; j < nVaryingB; ++j)
               for (int l = 0; l < paramDimensionB; ++l, ++pos)
                  Jt_e[pos] = Bt_e[j][l];
            for (int l = 0; l < nVaryingC; ++l, ++pos)
               Jt_e[pos] = Ct_e[l];

//                cout << "Jt_e = ";
//                for (int k = 0; k < Jt_e.size(); ++k) cout << Jt_e[k] << " ";
//                cout << endl;

            if (this->applyGradientStoppingCriteria(norm_Linf(Jt_e)))
            {
               status = LEVENBERG_OPTIMIZER_CONVERGED;
               goto end;
            }

            // The lhs J^t*J consists of several parts:
            //         [ U     W   X ]
            // J^t*J = [ W^t   V   Y ]
            //         [ X^t  Y^t  Z ],
            // where U, V and W are block-sparse matrices (due to the sparsity of A and B).
            // X, Y and Z contain only a few columns (the number of global parameters).
            for (int i = 0; i < nVaryingA; ++i) makeZeroMatrix(_Ui[i]);
            for (int j = 0; j < nVaryingB; ++j) makeZeroMatrix(_Vj[j]);
            for (size_t n = 0; n < Wn.count(); ++n) makeZeroMatrix(Wn[n]);
            makeZeroMatrix(_X);
            makeZeroMatrix(_Y);
            makeZeroMatrix(_Z);

            for (int obj = 0; obj < nObjs; ++obj)
               _costFunctions[obj]->fillHessian(_jointIndexW[obj], _Ui, _Vj, Wn, _X, _Y, _Z);

            if (currentIteration == 0)
            {
               // Initialize lambda as tau*max(JtJ[i][i])
               double maxEl = -1e30;
               if (optimizerVerbosenessLevel < 2)
               {
                  if (nVaryingA > 0)
                  {
                     for (int i = 0; i < nVaryingA; ++i)
                        for (int l = 0; l < paramDimensionA; ++l)
                           maxEl = std::max(maxEl, _Ui[i][l][l]);
                  }
                  if (nVaryingB > 0)
                  {
                     for (int j = 0; j < nVaryingB; ++j)
                        for (int l = 0; l < paramDimensionB; ++l)
                           maxEl = std::max(maxEl, _Vj[j][l][l]);
                  }
                  if (nVaryingC > 0)
                  {
                     for (int l = 0; l < nVaryingC; ++l)
                        maxEl = std::max(maxEl, _Z[l][l]);
                  }
               }
               else
               {
                  char   maxElSection = 'X';
                  int    maxElIndex = -1;
                  int    maxElRow = -1;

                  if (nVaryingA > 0)
                  {
                     for (int i = 0; i < nVaryingA; ++i)
                        for (int l = 0; l < paramDimensionA; ++l)
                        {
                           if (_Ui[i][l][l] > maxEl)
                           {
                              maxEl = _Ui[i][l][l];
                              maxElSection = 'U'; maxElIndex = i; maxElRow = l;
                           }
                        }
                  }
                  if (nVaryingB > 0)
                  {
                     for (int j = 0; j < nVaryingB; ++j)
                        for (int l = 0; l < paramDimensionB; ++l)
                        {
                           if (_Vj[j][l][l] > maxEl)
                           {
                              maxEl = _Vj[j][l][l];
                              maxElSection = 'V'; maxElIndex = j; maxElRow = l;
                           }
                        }
                  }
                  if (nVaryingC > 0)
                  {
                     for (int l = 0; l < nVaryingC; ++l)
                     {
                        if (_Z[l][l] > maxEl)
                        {
                           maxEl = _Z[l][l];
                           maxElSection = 'Z'; maxElIndex = 0; maxElRow = l;
                        }
                     }
                  }
                  cout << "ExtSparseLevenbergOptimizer: initial lambda = " << tau*maxEl
                       << "; max diagonal element found at " << maxElSection << "[" << maxElIndex << "]["
                       << maxElRow << "]" << endl;
               } // end if

               lambda = tau * maxEl;
            } // end if (currentIteration == 0)
         } // end if (computeDerivatives)

         for (int i = 0; i < nVaryingA; ++i)
         {
            for (int l = 0; l < paramDimensionA; ++l) diagUi[i][l] = _Ui[i][l][l];
         } // end for (i)

         for (int j = 0; j < nVaryingB; ++j)
         {
            for (int l = 0; l < paramDimensionB; ++l) diagVj[j][l] = _Vj[j][l][l];
         } // end for (j)

         for (int l = 0; l < nVaryingC; ++l) diagZ[l] = _Z[l][l];

         // Augment the diagonals with lambda (either by the standard additive update or by multiplication).
#if !defined(USE_MULTIPLICATIVE_UPDATE)
         for (int i = 0; i < nVaryingA; ++i)
            for (int l = 0; l < paramDimensionA; ++l)
               _Ui[i][l][l] += lambda;

         for (int j = 0; j < nVaryingB; ++j)
            for (int l = 0; l < paramDimensionB; ++l)
               _Vj[j][l][l] += lambda;

         for (int l = 0; l < nVaryingC; ++l)
            _Z[l][l] += lambda;
#else
         for (int i = 0; i < nVaryingA; ++i)
            for (unsigned l = 0; l < paramDimensionA; ++l)
               _Ui[i][l][l] = std::max(_Ui[i][l][l] * (1.0 + lambda), 1e-15);

         for (int j = 0; j < nVaryingB; ++j)
            for (unsigned l = 0; l < paramDimensionB; ++l)
               _Vj[j][l][l] = std::max(_Vj[j][l][l] * (1.0 + lambda), 1e-15);

         for (unsigned l = 0; l < nVaryingC; ++l)
            _Z[l][l] = std::max(_Z[l][l] * (1.0 + lambda), 1e-15);
#endif

         this->fillJtJ(_Ui, _Vj, Wn, _Z, _X, _Y);

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
                  cout << "ExtSparseLevenbergOptimizer: LDL decomposition failed. Increasing lambda." << endl;
               success = false;
            }
         }

         if (success)
         {
            double const deltaSqrLength = sqrNorm_L2(delta);
            double const paramLength = this->getParameterLength();

            if (optimizerVerbosenessLevel >= 2)
               cout << "ExtSparseLevenbergOptimizer: ||delta|| = " << sqrt(deltaSqrLength) << " ||paramLength|| = " << paramLength << endl;

            if (this->applyUpdateStoppingCriteria(paramLength, sqrt(deltaSqrLength)))
            {
               status = LEVENBERG_OPTIMIZER_SMALL_UPDATE;
               goto end;
            }

            // Copy the updates from delta to the respective arrays
            int pos = 0;

            for (int i = 0; i < nNonvaryingA; ++i) makeZeroVector(deltaAi[i]);
            for (int i = nNonvaryingA; i < nParametersA; ++i)
               for (int l = 0; l < paramDimensionA; ++l, ++pos)
                  deltaAi[i][l] = delta[pos];

            for (int j = 0; j < nNonvaryingB; ++j) makeZeroVector(deltaBj[j]);
            for (int j = nNonvaryingB; j < nParametersB; ++j)
               for (int l = 0; l < paramDimensionB; ++l, ++pos)
                  deltaBj[j][l] = delta[pos];

            makeZeroVector(deltaC);
            for (int l = nNonvaryingC; l < paramDimensionC; ++l, ++pos)
               deltaC[l] = delta[pos];

            saveAllParameters();
            if (nVaryingA > 0) updateParametersA(deltaAi);
            if (nVaryingB > 0) updateParametersB(deltaBj);
            if (nVaryingC > 0) updateParametersC(deltaC);

            if (optimizerVerbosenessLevel >= 2)
               cout << "ExtSparseLevenbergOptimizer: |deltaAi|^2 = " << squaredResidual(deltaAi) << " |deltaBj|^2 = " << squaredResidual(deltaBj) << endl;

            // cout << "deltaAi = ";
            // for (int i = 0; i < nParametersA; ++i)
            //    displayVector(deltaAi[i]);

            double newErr = 0.0;
            for (int obj = 0; obj < nObjs; ++obj)
            {
               SparseLM_CostFunction& costFun = *_costFunctions[obj];
               costFun.evalResidual(costFun._residuals);
               costFun.fillWeights(costFun._residuals, costFun._weights);
               for (int k = 0; k < costFun._nMeasurements; ++k)
                  scaleVectorIP(costFun._weights[k], costFun._residuals[k]);

               newErr += squaredResidual(costFun._residuals);
            } // end for (obj)

            rho = err - newErr;
            if (optimizerVerbosenessLevel >= 2)
               cout << "ExtSparseLevenbergOptimizer: |new residual|^2 = " << newErr << " rho = " << rho << endl;

#if !defined(USE_MULTIPLICATIVE_UPDATE)
            double const denom1 = lambda * deltaSqrLength;
#else
            double denom1 = 0.0f;
            for (int i = nNonvaryingA; i < nParametersA; ++i)
               for (int l = 0; l < paramDimensionA; ++l)
                  denom1 += deltaAi[i][l] * deltaAi[i][l] * diagUi[i-nNonvaryingA][l];

            for (int j = nNonvaryingB; j < nParametersB; ++j)
               for (int l = 0; l < paramDimensionB; ++l)
                  denom1 += deltaBj[j][l] * deltaBj[j][l] * diagVj[j-nNonvaryingB][l];

            for (int l = nNonvaryingC; l < paramDimensionC; ++l)
               denom1 += deltaC[l] * deltaC[l] * diagZ[l-nNonvaryingC];

            denom1 *= lambda;
#endif
            double const denom2 = innerProduct(delta, Jt_e);
            rho = rho / (denom1 + denom2);
            if (optimizerVerbosenessLevel >= 2)
               cout << "ExtSparseLevenbergOptimizer: rho = " << rho
                    << " denom1 = " << denom1 << " denom2 = " << denom2 << endl;
         } // end if (success)

         if (success && rho > 0)
         {
            if (optimizerVerbosenessLevel >= 2)
               cout << "ExtSparseLevenbergOptimizer: Improved solution - decreasing lambda." << endl;
            // Improvement in the new solution
            decreaseLambda(rho);
            computeDerivatives = true;
         }
         else
         {
            if (optimizerVerbosenessLevel >= 2)
               cout << "ExtSparseLevenbergOptimizer: Inferior solution - increasing lambda." << endl;
            restoreAllParameters();
            increaseLambda();
            computeDerivatives = false;

            // Restore diagonal elements in Ui, Vj and Z.
            for (int i = 0; i < nVaryingA; ++i)
            {
               for (int l = 0; l < paramDimensionA; ++l) _Ui[i][l][l] = diagUi[i][l];
            } // end for (i)

            for (int j = 0; j < nVaryingB; ++j)
            {
               for (int l = 0; l < paramDimensionB; ++l) _Vj[j][l][l] = diagVj[j][l];
            } // end for (j)

            for (int l = 0; l < nVaryingC; ++l) _Z[l][l] = diagZ[l];
         } // end if
      } // end for (currentIteration)

     end:;
      if (optimizerVerbosenessLevel >= 2)
         cout << "Leaving ExtSparseLevenbergOptimizer::minimize()." << endl;
   } // end ExtSparseLevenbergOptimizer::minimize()

#endif // defined(V3DLIB_ENABLE_SUITESPARSE)

} // end namespace V3D
