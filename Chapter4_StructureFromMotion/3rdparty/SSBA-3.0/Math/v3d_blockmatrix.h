// -*- C++ -*-
#ifndef V3D_BLOCK_MATRIX_H
#define V3D_BLOCK_MATRIX_H

#include "Math/v3d_linear.h"
#include "Math/v3d_linear_ldlt.h"

namespace V3D
{

   template <typename Elem> struct DiagonalBlockMatrix;

   template <typename Elem>
   inline void
   makeZeroMatrix(DiagonalBlockMatrix<Elem>& dst);

   template <typename Elem>
   inline void
   addMatrices(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem> const& B, DiagonalBlockMatrix<Elem>& dst);

   template <typename Elem>
   inline void
   scaleMatrix(DiagonalBlockMatrix<Elem> const& A, Elem scale, DiagonalBlockMatrix<Elem>& dst);

   template <typename Elem>
   inline void
   multiply_A_B(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem> const& B, DiagonalBlockMatrix<Elem>& dst);

   template <typename Elem>
   inline void
   multiply_A_Bt(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem> const& B, DiagonalBlockMatrix<Elem>& dst);

   template <typename Elem>
   inline void
   multiply_At_A(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem>& dst);

   // This is a matrix consisting of diagonal matrices of the same size as blocks:
   // A = [ D_11 D_12 ... D_1N ]
   //     [ D_21 D_22 ... D_2N ]
   //     [  ...               ]
   //     [ D_M1 D_M2 ... D_MN ]
   // This is not the same as a block diagonal matrix!
   template <typename Elem>
   struct DiagonalBlockMatrix
   {
         DiagonalBlockMatrix(int const M, int const N, int const P)
            : _M(M), _N(N), _P(P)
         {
            _values = new Elem[_M*_N*_P];
            //std::fill(_values, _values + _M*_N*_P, Elem(0));
         }

         ~DiagonalBlockMatrix()
         {
            delete [] _values;
         }

         int const getM() const { return _M; }
         int const getN() const { return _N; }
         int const getP() const { return _P; }

         int const num_rows() const { return _M*_P; }
         int const num_cols() const { return _N*_P; }

         template <typename Vec>
         void getBlock(int const m, int const n, Vec& dst) const
         {
            assert(dst.size() == _P);

            int const ofs = this->getBlockOfs(m, n);
            for (int i = 0; i < _P; ++i) dst[i] = _values[ofs + i];
         }

         Elem& elemAt(int const m, int const n, int const p)
         {
            return _values[this->getBlockOfs(m, n) + p];
         }

         Elem const& elemAt(int const m, int const n, int const p) const
         {
            return _values[this->getBlockOfs(m, n) + p];
         }

         Elem operator()(int const row1, int const col1) const
         {
            int const row = row1 - 1;
            int const col = col1 - 1;
            int const m = row / _P;
            int const n = col / _P;
            int const pr = row - m*_P;
            int const pc = col - n*_P;

            if (pr != pc) return 0;
            return this->elemAt(m, n, pr);
         }

         Elem * getBlock(int const m, int const n)
         {
            return _values + this->getBlockOfs(m, n);
         }

         Elem const * getBlock(int const m, int const n) const
         {
            return _values + this->getBlockOfs(m, n);
         }

         void augmentDiagonal(Elem const mu)
         {
            for (int k = 0; k < std::min(_M, _N); ++k)
            {
               int const ofs = this->getBlockOfs(k, k);
               for (int i = 0; i < _P; ++i) _values[ofs + i] += mu;
            }
         } // end augmentDiagonal()

         template <typename Vec>
         void scaleRows(Vec const& w)
         {
            assert(w.size() == this->num_rows());

            for (int m = 0; m < _M; ++m)
               for (int p = 0; p < _P; ++p)
               {
                  Elem const s = w[m*_P + p];
                  for (int n = 0; n < _N; ++n)
                     this->getBlock(m, n)[p] *= s;
               } // end for (p)
         } // end scaleRows()

         template <typename Vec>
         void scaleColumns(Vec const& w)
         {
            assert(w.size() == this->num_cols());

            for (int n = 0; n < _N; ++n)
               for (int p = 0; p < _P; ++p)
               {
                  Elem const s = w[n*_P + p];
                  for (int m = 0; m < _M; ++m)
                     this->getBlock(m, n)[p] *= s;
               } // end for (p)
         } // end scaleRows()

         int getColumnNonzeroCount(unsigned int col) const { return _M; }

         template <typename VecA, typename VecB>
         void getSparseColumn(unsigned int col, VecA& rows, VecB& values) const
         {
            int const n = col / _P;
            int const p = col - n*_P;

            for (int m = 0; m < _M; ++m)
            {
               rows[m] = m*_P + p;
               values[m] = this->elemAt(m, n, p);
            }
         }

         int getRowNonzeroCount(unsigned int row) const { return _N; }

         template <typename VecA, typename VecB>
         void getSparseRow(unsigned int row, VecA& cols, VecB& values) const
         {
            int const m = row / _P;
            int const p = row - m*_P;

            for (int n = 0; n < _N; ++n)
            {
               cols[n] = n*_P + p;
               values[n] = this->elemAt(m, n, p);
            }
         }

      protected:
         int getBlockOfs(int const m, int const n) const
         {
            return (m*_N + n)*_P;
         }

         int const _M, _N, _P;
         Elem    * _values;

         friend void makeZeroMatrix<>(DiagonalBlockMatrix<Elem>& dst);
         friend void addMatrices<>(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem> const& B, DiagonalBlockMatrix<Elem>& dst);
         friend void scaleMatrix<>(DiagonalBlockMatrix<Elem> const& A, Elem scale, DiagonalBlockMatrix<Elem>& dst);
         friend void multiply_A_B<>(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem> const& B, DiagonalBlockMatrix<Elem>& dst);
         friend void multiply_A_Bt<>(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem> const& B, DiagonalBlockMatrix<Elem>& dst);
         friend void multiply_At_A<>(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem>& dst);
   }; // end struct DiagonalBlockMatrix

   template <typename Elem>
   inline void
   makeZeroMatrix(DiagonalBlockMatrix<Elem>& dst)
   {
      std::fill(dst._values, dst._values + dst._M*dst._N*dst._P, Elem(0));
   }

   template <typename Elem>
   inline void
   addMatrices(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem> const& B, DiagonalBlockMatrix<Elem>& dst)
   {
      assert(A._P == B._P);
      assert(A._P == dst._P);
      assert(A._M == B._M);
      assert(A._N == B._N);
      assert(A._M == dst._M);
      assert(A._N == dst._N);

      int const sz = A._P * A._M * A._N;
      for (int i = 0; i < sz; ++i) dst._values[i] = A._values[i] + B._values[i];
   } // end addMatrices()

   template <typename Elem>
   inline void
   scaleMatrix(DiagonalBlockMatrix<Elem> const& A, Elem scale, DiagonalBlockMatrix<Elem>& dst)
   {
      assert(A._P == dst._P);
      assert(A._M == dst._M);
      assert(A._N == dst._N);

      int const sz = A._P * A._M * A._N;
      for (int i = 0; i < sz; ++i) dst._values[i] = scale * A._values[i];
   } // end scaleMatrix()

   template <typename Elem>
   inline void
   multiply_A_B(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem> const& B, DiagonalBlockMatrix<Elem>& dst)
   {
      assert(A._P == B._P);
      assert(A._P == dst._P);
      assert(A._N == B._M);
      assert(A._M == dst._M);
      assert(B._N == dst._N);

      makeZeroMatrix(dst);
      for (int i = 0; i < dst._M; ++i)
         for (int j = 0; j < dst._N; ++j)
         {
            Elem * valsC = dst.getBlock(i, j);

            for (int k = 0; k < A._N; ++k)
            {
               Elem const * valsA = A.getBlock(i, k);
               Elem const * valsB = B.getBlock(k, j);

               for (int p = 0; p < A._P; ++p) valsC[p] += valsA[p] * valsB[p];
            } // end for (k)
         } // end for (j)
   } // end multiply_A_B()

   template <typename Elem>
   inline void
   multiply_A_Bt(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem> const& B, DiagonalBlockMatrix<Elem>& dst)
   {
      assert(A._P == B._P);
      assert(A._P == dst._P);
      assert(A._N == B._N);
      assert(A._M == dst._M);
      assert(B._M == dst._N);

      makeZeroMatrix(dst);
      for (int i = 0; i < dst._M; ++i)
         for (int j = 0; j < dst._N; ++j)
         {
            Elem * valsC = dst.getBlock(i, j);

            for (int k = 0; k < A._N; ++k)
            {
               Elem const * valsA = A.getBlock(i, k);
               Elem const * valsB = B.getBlock(j, k);

               for (int p = 0; p < A._P; ++p) valsC[p] += valsA[p] * valsB[p];
            } // end for (k)
         } // end for (j)
   } // end multiply_A_B()

   template <typename Elem>
   inline void
   multiply_At_A(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem>& dst)
   {
      assert(A._P == dst._P);
      assert(A._N == dst._M);
      assert(A._N == dst._N);

      makeZeroMatrix(dst);
      for (int i = 0; i < A._N; ++i)
         for (int j = 0; j < A._N; ++j)
         {
            Elem * valsC = dst.getBlock(i, j);

            for (int k = 0; k < A._M; ++k)
            {
               Elem const * valsAt = A.getBlock(k, i);
               Elem const * valsA  = A.getBlock(k, j);

               for (int p = 0; p < A._P; ++p) valsC[p] += valsAt[p] * valsA[p];
            } // end for (k)
         } // end for (j)
   } // end multiply_At_A()

   template <typename Elem, typename MatB, typename MatC>
   inline void
   multiply_A_B(DiagonalBlockMatrix<Elem> const& A, MatB const& B, MatC& dst)
   {
      assert(B.num_rows() == A.num_cols());
      assert(dst.num_rows() == A.num_rows());
      assert(dst.num_cols() == B.num_cols());

      int const M = A.getM();
      int const N = A.getN();
      int const P = A.getP();

      Vector<Elem> accums(P);

      for (int i = 0; i < M; ++i)
      {
         int const row0 = i*P;

         for (int dstCol = 0; dstCol < dst.num_cols(); ++ dstCol)
         {
            makeZeroVector(accums);

            for (int j = 0; j < N; ++j)
            {
               int const col0 = j*P;
               Elem const * valsA = A.getBlock(i, j);
               for (int p = 0; p < P; ++p) accums[p] += valsA[p] * B[col0 + p][dstCol];
            } // end for (j)
            for (int p = 0; p < P; ++p) dst[row0 + p][dstCol] = accums[p];
         } // end for (dstCol)
      } // end for (i)
   } // end multiply_A_B()


   template <typename Elem, typename VecA, typename VecB>
   inline void
   multiply_A_v(DiagonalBlockMatrix<Elem> const& A, VecA const& v, VecB& dst)
   {
      assert(v.size() == A.num_cols());
      assert(dst.size() == A.num_rows());

      int const M = A.getM();
      int const N = A.getN();
      int const P = A.getP();

      makeZeroVector(dst);

      for (int i = 0; i < M; ++i)
      {
         int const row0 = i*P;
         for (int j = 0; j < N; ++j)
         {
            int const col0 = j*P;
            Elem const * valsA = A.getBlock(i, j);
            for (int p = 0; p < P; ++p) dst[row0 + p] += valsA[p] * v[col0 + p];
         } // end for (j)
      } // end for (i)
   } // end multiply_A_v()

   template <typename Elem, typename VecA, typename VecB>
   inline void
   multiply_At_v(DiagonalBlockMatrix<Elem> const& A, VecA const& v, VecB& dst)
   {
      assert(v.size() == A.num_rows());
      assert(dst.size() == A.num_cols());

      int const M = A.getM();
      int const N = A.getN();
      int const P = A.getP();

      makeZeroVector(dst);

      for (int j = 0; j < N; ++j)
      {
         int const row0 = j*P;
         for (int i = 0; i < M; ++i)
         {
            int const col0 = i*P;
            Elem const * valsA = A.getBlock(i, j);
            for (int p = 0; p < P; ++p) dst[row0 + p] += valsA[p] * v[col0 + p];
         } // end for (i)
      } // end for (j)
   } // end multiply_At_v()

   template <typename Elem>
   struct DiagonalBlock_LDL
   {
         DiagonalBlock_LDL(DiagonalBlockMatrix<Elem> const& A)
            : _L(A.getM(), A.getN(), A.getP()),
              _D(A.num_rows(), Elem(0))
         {
            if (A.num_rows() != A.num_cols())
               throwV3DErrorHere("DiagonalBlock_LDL(): matrix is not square.");

            int const N = A.getN();
            int const P = A.getP();

            makeZeroMatrix(_L);
            for (int n = 0; n < N; ++n)
               for (int p = 0; p < P; ++p)
                  _L.elemAt(n, n, p) = 1;

            for (int n = 0; n < N; ++n)
            {
               Elem const * diagA = A.getBlock(n, n);
               for (int p = 0; p < P; ++p)
               {
                  int const j = n*P + p;

                  // First, compute the current element of D
                  Elem d = diagA[p];
                  for (int l = 0; l < n; ++l)
                  {
                     int const col = l*P + p;
                     Elem const Ljk = _L.elemAt(n, l, p);
                     d -= Ljk*Ljk*_D[col];
                  } // end for (l)
                  _D[j] = d;

                  // Update L below position (j,j)
                  // Note: only rows j+l*P are nonzero in L!
                  for (int m = n+1; m < N; ++m)
                  {
                     Elem Lij = A.elemAt(m, n, p); // Aij
                     for (int l = 0; l < n; ++l)
                     {
                        int const col = l*P + p;
                        Elem const Lik = _L.elemAt(m, l, p);
                        Elem const Ljk = _L.elemAt(n, l, p);
                        Lij -= Lik*Ljk*_D[col];
                     }
                     Lij /= d;
                     _L.elemAt(m, n, p) = Lij;
                  } // end for (m)
               } // end for (p)
            } // end for (n)
         } // end DiagonalBlock_LDL()

         DiagonalBlockMatrix<Elem> const& getL() const { return _L; }
         std::vector<Elem> const& getD() const { return _D; }

      protected:
         // This routine is not needed anymore
         // int extractSparseLRow(int const m, int const p, std::vector<Elem>& vals, std::vector<int>& cols) const
         // {
         //    int const P = _L.getP();

         //    int const nnz = m; // note, L is lower diagonal, and we have to ignore the main diagonal (which is 1 for LDL^T decomposition)

         //    for (int i = 0; i < nnz; ++i)
         //    {
         //       cols[i] = P*i + p;
         //       vals[i] = _L.getBlock(m, i)[p];
         //    }

         //    return nnz;
         // } // end extractSparseLRow()

         DiagonalBlockMatrix<Elem> _L;
         std::vector<double> _D;
   }; // end struct DiagonalBlock_LDL

   template <typename Elem>
   inline void
   makeInvertedMatrix_PSD(DiagonalBlockMatrix<Elem> const& A, DiagonalBlockMatrix<Elem>& dst)
   {
      assert(dst.getN() == A.getN());
      assert(dst.getM() == A.getM());
      assert(dst.getP() == A.getP());

      int const N = A.getN();
      int const P = A.getP();

      Matrix<Elem> Ai(N, N);
      Matrix<Elem> I(N, N); makeIdentityMatrix(I);

      for (int p = 0; p < P; ++p)
      {
         for (int m = 0; m < N; ++m)
            for (int n = 0; n < N; ++n)
               Ai[m][n] = A.elemAt(m, n, p);

#if 0
         Cholesky<Elem> chol(Ai);
         Ai = chol.solve(I);
#else
         LDLt<Elem> ldlt(Ai);
         Ai = ldlt.solveMat(I);
#endif

         for (int m = 0; m < N; ++m)
            for (int n = 0; n < N; ++n)
               dst.elemAt(m, n, p) = Ai[m][n];
      } // end for (p)
   } // end makeInvertedMatrix()

   template <typename Elem>
   inline Elem
   matrixNorm_Linf(DiagonalBlockMatrix<Elem> const& A)
   {
      Elem res = 0;
      for (int i = 0; i < A.getM(); ++i)
         for (int p = 0; p < A.getP(); ++p)
         {
            Elem rowSum = 0;
            for (int j = 0; j < A.getN(); ++j)
            {
               Elem const * vals = A.getBlock(i, j);
               rowSum += fabs(vals[p]);
            } // end for (j)
            res = std::max(res, rowSum);
         } // end for (p)
      return res;
   } // end matrixNorm_Linf()

   template <typename Elem>
   inline Elem
   matrixNorm_L1(DiagonalBlockMatrix<Elem> const& A)
   {
      Elem res = 0;
      for (int j = 0; j < A.getN(); ++j)
         for (int p = 0; p < A.getP(); ++p)
         {
            Elem colSum = 0;
            for (int i = 0; i < A.getM(); ++i)
            {
               Elem const * vals = A.getBlock(i, j);
               colSum += fabs(vals[p]);
            } // end for (i)
            res = std::max(res, colSum);
         } // end for (p)
      return res;
   } // end matrixNorm_L1()

   template <typename Elem>
   inline void
   displayMatrix(DiagonalBlockMatrix<Elem> const& A)
   {
      using namespace std;

      cout << "[ ";
      for (int r = 0; r < A.num_rows(); ++r)
      {
         for (int c = 0; c < A.num_cols(); ++c)
            cout << A(r+1, c+1) << " ";
         if (r < A.num_rows()-1)
            cout << endl;
         else
            cout << "]" << endl;
      }
   } // end displayMatrix()

//======================================================================

   // Matrix with count MxN nonzero blocks along the diagonal (the standard block-diagonal matrix)
   template <typename Elem>
   struct BlockDiagonalMatrix
   {
         BlockDiagonalMatrix(int const count, int const M, int const N)
            : _count(count), _M(M), _N(N), _array(count, M, N)
         { }

         int const num_rows() const { return _count*_M; }
         int const num_cols() const { return _count*_N; }

         int const getM() const { return _M; }
         int const getN() const { return _N; }
         int const getCount() const { return _count; }

         Matrix<Elem> const& getBlock(int ix) const { return _array[ix]; }
         Matrix<Elem>&       getBlock(int ix)       { return _array[ix]; }

         int getColumnNonzeroCount(unsigned int col) const { return _M; }

         template <typename VecA, typename VecB>
         void getSparseColumn(unsigned int col, VecA& rows, VecB& values) const
         {
            int const ix = col / _N;
            int const j0 = ix * _N;
            int const i0 = ix * _M;
            int const j  = col - j0;

            Matrix<Elem> const& A = _array[ix];

            for (int i = 0; i < _M; ++i)
            {
               rows[i] = i + i0;
               values[i] = A[i][j];
            }
         } // end getSparseColumn()

         int getRowNonzeroCount(unsigned int row) const { return _N; }

         template <typename VecA, typename VecB>
         void getSparseRow(unsigned int row, VecA& cols, VecB& values) const
         {
            int const ix = row / _M;
            int const j0 = ix * _N;
            int const i0 = ix * _M;
            int const i  = row - i0;

            Matrix<Elem> const& A = _array[ix];

            for (int j = 0; j < _N; ++j)
            {
               cols[j] = j + j0;
               values[j] = A[i][j];
            }
         } // end getSparseRow()

      protected:
         int const _count, _M, _N;
         MatrixArray<Elem> _array;
   }; // end struct BlockDiagonalMatrix

   template <typename Elem, typename VecA, typename VecB>
   inline void
   multiply_A_v(BlockDiagonalMatrix<Elem> const& A, VecA const& v, VecB& dst)
   {
      assert(v.size() == A.num_cols());
      assert(dst.size() == A.num_rows());

      int const count = A.getCount();
      int const M     = A.getM();
      int const N     = A.getN();

      makeZeroVector(dst);

      for (int b = 0; b < count; ++b)
      {
         int const row0 = b*M;
         int const col0 = b*N;

         Matrix<double> const& Asub = A.getBlock(b);

         for (int i = 0; i < M; ++i)
         {
            for (int j = 0; j < N; ++j)
               dst[row0 + i] += Asub[i][j] * v[col0 + j];
         } // end for (i)
      } // end for (b)
   } // end multiply_A_v()

   template <typename Elem, typename VecA, typename VecB>
   inline void
   multiply_At_v(BlockDiagonalMatrix<Elem> const& A, VecA const& v, VecB& dst)
   {
      assert(v.size() == A.num_rows());
      assert(dst.size() == A.num_cols());

      int const count = A.getCount();
      int const M     = A.getM();
      int const N     = A.getN();

      makeZeroVector(dst);

      for (int b = 0; b < count; ++b)
      {
         int const row0 = b*N;
         int const col0 = b*M;

         Matrix<double> const& Asub = A.getBlock(b);

         for (int i = 0; i < N; ++i)
         {
            for (int j = 0; j < M; ++j)
               dst[row0 + i] += Asub[j][i] * v[col0 + j];
         } // end for (i)
      } // end for (b)
   } // end multiply_At_v()

   // Matrixproduct of a diagonal block with a dense matrix
   template <typename Elem, typename MatB, typename MatC>
   inline void
   multiply_A_B(BlockDiagonalMatrix<Elem> const& A, MatB const& B, MatC& dst)
   {
      assert(B.num_rows() == A.num_cols());
      assert(dst.num_rows() == A.num_rows());
      assert(dst.num_cols() == B.num_cols());

      typedef typename MatC::value_type ElemDst;

      int const count = A.getCount();
      int const M     = A.getM();
      int const N     = A.getN();

      for (int b = 0; b < count; ++b)
      {
         int const row0 = b*M;
         int const col0 = b*N;

         Matrix<double> const& Asub = A.getBlock(b);

         for (int i = 0; i < M; ++i)
         {
            for (int j = 0; j < dst.num_cols(); ++j)
            {
               ElemDst accum = 0;
               for (int k = 0; k < N; ++k) accum += Asub[i][k] * B[col0 + k][j];
               dst[row0 + i][j] = accum;
            }
         } // end for (i)
      } // end for (b)
   } // end multiply_A_B()

   // Matrixproduct of a diagonal block with a dense matrix
   template <typename Elem, typename MatB, typename MatC>
   inline void
   multiply_A_Bt(BlockDiagonalMatrix<Elem> const& A, MatB const& B, MatC& dst)
   {
      assert(B.num_cols() == A.num_cols());
      assert(dst.num_rows() == A.num_rows());
      assert(dst.num_cols() == B.num_rows());

      typedef typename MatC::value_type ElemDst;

      int const count = A.getCount();
      int const M     = A.getM();
      int const N     = A.getN();

      for (int b = 0; b < count; ++b)
      {
         int const row0 = b*M;
         int const col0 = b*N;

         Matrix<double> const& Asub = A.getBlock(b);

         for (int i = 0; i < M; ++i)
         {
            for (int j = 0; j < dst.num_cols(); ++j)
            {
               ElemDst accum = 0;
               for (int k = 0; k < N; ++k) accum += Asub[i][k] * B[j][col0 + k];
               dst[row0 + i][j] = accum;
            }
         } // end for (i)
      } // end for (b)
   } // end multiply_A_Bt()

   // Matrixproduct of two block diagonal matrices yielding a block diagonal matrix
   template <typename Elem>
   inline void
   multiply_A_Bt(BlockDiagonalMatrix<Elem> const& A, BlockDiagonalMatrix<Elem> const& B, BlockDiagonalMatrix<Elem>& dst)
   {
      assert(A.getCount() == B.getCount());
      assert(A.getCount() == dst.getCount());
      assert(A.getM() == dst.getM());
      assert(B.getM() == dst.getN());
      assert(A.getN() == B.getN());

      int const count = dst.getCount();

      for (int k = 0; k < count; ++k)
      {
         Matrix<double> const& Ak = A.getBlock(k);
         Matrix<double> const& Bk = B.getBlock(k);
         Matrix<double>& Ck = dst.getBlock(k);
         multiply_A_Bt(Ak, Bk, Ck);
      } // end for (k)
   } // end multiply_A_Bt()

   // Matrixproduct of two block diagonal matrices yielding a block diagonal matrix
   template <typename Elem>
   inline void
   multiply_A_At(BlockDiagonalMatrix<Elem> const& A, BlockDiagonalMatrix<Elem>& dst)
   {
      assert(A.getCount() == dst.getCount());
      assert(A.getM() == dst.getM());
      assert(A.getM() == dst.getN());

      int const count = dst.getCount();

      for (int k = 0; k < count; ++k)
      {
         Matrix<double> const& Ak = A.getBlock(k);
         Matrix<double>& Ck = dst.getBlock(k);
         multiply_A_At(Ak, Ck);
      } // end for (k)
   } // end multiply_A_At()

   // Multiplication of certain Kronecker-type matrices with block diagonal matrices

   template <typename Elem, typename MatA, typename MatB, typename MatD, typename MatE, typename MatDst>
   inline void
   multiply_AxB_C_DxE(MatA const& A, MatB const& B, BlockDiagonalMatrix<Elem> const& C, MatD const& D, MatE const& E, MatDst& dst)
   {
      int const N = C.getCount();

      assert(A.num_cols() == N);
      assert(D.num_rows() == N);

      int const mA = A.num_rows();
      int const nA = A.num_cols();
      int const mB = B.num_rows();
      int const nB = B.num_cols();
      int const mC = C.getM();
      int const nC = C.getN();
      int const mD = D.num_rows();
      int const nD = D.num_cols();
      int const mE = E.num_rows();
      int const nE = E.num_cols();

      assert(C.getN() == mE);

      Matrix<Elem> CiE(mC, nE);
      Matrix<Elem> BCiE(mB, nE);
      Matrix<Elem> accum(mB, nE);

      MatrixArray<Elem> BCE(N, mB, nE);

      for (int i = 0; i < N; ++i)
      {
         Matrix<Elem> const& Ci = C.getBlock(i);
         multiply_A_B(Ci, E, CiE);
         multiply_A_B(B, CiE, BCE[i]);
      } // end for (i)

      for (int k1 = 0; k1 < mA; ++k1)
      {
         int const dstRow0 = k1*mB;
         for (int k2 = 0; k2 < nD; ++k2)
         {
            int const dstCol0 = k2*nE;

            makeZeroMatrix(accum);

            for (int i = 0; i < N; ++i)
            {
               scaleMatrix(BCE[i], A[k1][i]*D[i][k2], BCiE);
               addMatricesIP(BCiE, accum);
            } // end for (i)
            copyMatrixSlice(accum, 0, 0, mB, nE, dst, dstRow0, dstCol0);
         } // end for (k2)
      } // end for (k1)
   } // end multiply_AxB_C_DxE()

   template <typename Elem, typename MatU, typename MatV, typename MatDst>
   inline void
   multiply_VtxU_C_VxUt(MatU const& U, MatV const& Vt, BlockDiagonalMatrix<Elem> const& C, MatDst& dst)
   {
      int const N = C.getCount();
      int const P = U.num_rows();
      int const K = Vt.num_rows();

      // We need to form U*Ci*Ut
      assert(U.num_cols() == C.getN());
      assert(U.num_cols() == C.getM());

      assert(Vt.num_cols() == N);

      // dst has KxK PxP blocks
      assert(dst.num_rows() == K*P);
      assert(dst.num_cols() == K*P);

      Matrix<Elem> CiUt(C.getM(), P);
      Matrix<Elem> UCiUt(P, P);
      Matrix<Elem> accum(P, P);

      MatrixArray<Elem> UCUt(N, P, P);

      for (int i = 0; i < N; ++i)
      {
         Matrix<Elem> const& Ci = C.getBlock(i);
         multiply_A_Bt(Ci, U, CiUt);
         multiply_A_B(U, CiUt, UCUt[i]);
      } // end for (i)

      for (int k1 = 0; k1 < K; ++k1)
      {
         int const dstRow0 = k1*P;
         for (int k2 = 0; k2 < K; ++k2)
         {
            int const dstCol0 = k2*P;

            makeZeroMatrix(accum);

            for (int i = 0; i < N; ++i)
            {
               scaleMatrix(UCUt[i], Vt[k1][i]*Vt[k2][i], UCiUt);
               addMatricesIP(UCiUt, accum);
            } // end for (i)
            copyMatrixSlice(accum, 0, 0, P, P, dst, dstRow0, dstCol0);
         } // end for (k2)
      } // end for (k1)
   } // end multiply_VtxU_C_VxUt()

   template <typename Elem, typename MatV, typename MatDst>
   inline void
   multiply_VtxI_C_VxI(MatV const& Vt, BlockDiagonalMatrix<Elem> const& C, MatDst& dst)
   {
      int const N = C.getCount();
      int const P = C.getM();
      int const K = Vt.num_rows();

      assert(P == C.getN());
      assert(Vt.num_cols() == N);

      // dst has KxK PxP blocks
      assert(dst.num_rows() == K*P);
      assert(dst.num_cols() == K*P);

      Matrix<Elem> UCiUt(P, P);
      Matrix<Elem> accum(P, P);

      for (int k1 = 0; k1 < K; ++k1)
      {
         int const dstRow0 = k1*P;
         for (int k2 = 0; k2 < K; ++k2)
         {
            int const dstCol0 = k2*P;

            makeZeroMatrix(accum);

            for (int i = 0; i < N; ++i)
            {
               Matrix<Elem> const& Ci = C.getBlock(i);
               scaleMatrix(Ci, Vt[k1][i]*Vt[k2][i], UCiUt);
               addMatricesIP(UCiUt, accum);
            } // end for (i)
            copyMatrixSlice(accum, 0, 0, P, P, dst, dstRow0, dstCol0);
         } // end for (k2)
      } // end for (k1)
   } // end multiply_VtxI_C_VxI()

   template <typename Elem, typename MatU, typename MatV, typename MatDst>
   inline void
   multiply_VtxU_C(MatU const& U, MatV const& Vt, BlockDiagonalMatrix<Elem> const& C, MatDst& dst)
   {
      int const N = C.getCount();
      int const P = U.num_rows();
      int const K = U.num_cols();

      assert(Vt.num_rows() == K);
      assert(Vt.num_cols() == N);

      assert(C.getN() == K);
      assert(C.getM() == K);

      Matrix<Elem> UCi(P, K);

      MatrixArray<Elem> UC(N, P, K);

      for (int i = 0; i < N; ++i)
         multiply_A_B(U, C.getBlock(i), UC[i]);

      for (int k = 0; k < K; ++k)
      {
         int const dstRow0 = k*P;
         for (int i = 0; i < N; ++i)
         {
            int const dstCol0 = i*K;

            copyMatrix(UC[i], UCi);
            scaleMatrixIP(Vt[k][i], UCi);
            copyMatrixSlice(UCi, 0, 0, P, K, dst, dstRow0, dstCol0);
         } // end for (i)
      } // end for (k)
   } // end multiply_VtxU_C_VxUt()

} // end namespace V3D

#endif
