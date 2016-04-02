// -*- C++ -*-
// This is a simple implementation of the LDL^T decomposition for PSD matrices

#ifndef V3D_LINEAR_LDLT_H
#define V3D_LINEAR_LDLT_H

#include "Math/v3d_linear.h"

namespace V3D
{

   template <typename Real>
   struct LDLt
   {
         LDLt(Matrix<Real> const& A)
            : _L(A.num_rows(), A.num_rows()), _D(A.num_rows())
         {
            if (A.num_rows() != A.num_cols()) throwV3DErrorHere("LDLt::LDLt(): matrix A is not square.");

            int const N = A.num_rows();

            makeIdentityMatrix(_L);

            for (int j = 0; j < N; ++j)
            {
               // First, compute the current element of D
               Real Dj = 0;
               for (int k = 0; k < j; ++k) Dj += _L[j][k]*_L[j][k]*_D[k];
               Dj = A[j][j] - Dj;
               _D[j] = Dj;

               // Update L below position (j,j)
               for (int i = j+1; i < N; ++i)
               {
                  Real Lij = 0;
                  for (int k = 0; k < j; ++k)
                     Lij += _L[i][k]*_L[j][k]*_D[k];
                  Lij = A[i][j] - Lij;
                  Lij /= Dj;
                  _L[i][j] = Lij;
               } // end for (i)
            } // end for (j)
         } // end LDLt()

         Matrix<Real> const& getL() const { return _L; }
         Vector<Real> const& getD() const { return _D; }

         template <typename Vec>
         Vector<Real> solveVec(Vec const& b) const
         {
            int const N = _L.num_rows();
            if (b.size() != N) throwV3DErrorHere("LDLt::solve(): size of vector b does not match.");

            Vector<Real> x(N);
            copyVector(b, x);

            // Solve L*y = b;
            for (int k = 0; k < N; ++k)
            {
               for (int i = 0; i < k; ++i) x[k] -= x[i]*_L[k][i];
            }

            for (int k = 0; k < N; ++k) x[k] /= _D[k];

            // Solve L'*X = Y;
            for (int k = N-1; k >= 0; --k)
            {
               for (int i = k+1; i < N; ++i) x[k] -= x[i]*_L[i][k];
            }

            return x;
         } // end solveVec()

         template <typename Mat>
         Matrix<Real> solveMat(Mat const& B) const
         {
            int const N = _L.num_rows();
            if (B.num_rows() != N) throwV3DErrorHere("LDLt::solve(): size of matrix B does not match.");

            int const K = B.num_cols();

            Matrix<Real> X(N, K);
            copyMatrix(B, X);

            for (int j = 0; j < K; ++j)
            {
               // Solve L*y = b;
               for (int k = 0; k < N; ++k)
               {
                  for (int i = 0; i < k; ++i) X[k][j] -= X[i][j]*_L[k][i];
               }

               for (int k = 0; k < N; ++k) X[k][j] /= _D[k];

               // Solve L'*X = Y;
               for (int k = N-1; k >= 0; --k)
               {
                  for (int i = k+1; i < N; ++i) X[k][j] -= X[i][j]*_L[i][k];
               }
            } // end for (j)

            return X;
         } // end solveMat()

      private:
         Matrix<Real> _L;
         Vector<Real> _D;
   }; // end struct LDLt

} // end namespace V3D

#endif
