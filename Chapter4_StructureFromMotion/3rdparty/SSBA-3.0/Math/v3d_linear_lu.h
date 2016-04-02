// -*- C++ -*-

#ifndef V3D_LINEAR_LU_H
#define V3D_LINEAR_LU_H

// This is adapted from the TNT matrix and linear algebra library, which is in the public domain.
// This file contains only the LU decomposition code.

#include "Math/v3d_linear.h"
#include "Base/v3d_exception.h"

namespace V3D
{



   /** LU Decomposition.
       <P>
       For an m-by-n matrix A with m >= n, the LU decomposition is an m-by-n
       unit lower triangular matrix L, an n-by-n upper triangular matrix U,
       and a permutation vector piv of length m so that A(piv,:) = L*U.
       If m < n, then L is m-by-m and U is m-by-n.
       <P>
       The LU decompostion with pivoting always exists, even if the matrix is
       singular, so the constructor will never fail.  The primary use of the
       LU decomposition is in the solution of square systems of simultaneous
       linear equations.  This will fail if isNonsingular() returns false.
   */
   template <class Real>
   class LU
   {

      private:

         /* Array for internal storage of decomposition.  */
         Matrix<Real>  LU_;
         int m, n, pivsign; 
         Vector<int> piv;


         Matrix<Real> permute_copy( const Matrix<Real> &A, 
                                    const Vector<int> &piv, int j0, int j1)
         {
            int piv_length = piv.size();

            Matrix<Real> X(piv_length, j1-j0+1);


            for (int i = 0; i < piv_length; i++) 
               for (int j = j0; j <= j1; j++) 
                  X[i][j-j0] = A[piv[i]][j];

            return X;
         }

         Vector<Real> permute_copy( const Vector<Real> &A, 
                                    const Vector<int> &piv)
         {
            int piv_length = piv.size();
            if (piv_length != A.size())
               return Vector<Real>();

            Vector<Real> x(piv_length);


            for (int i = 0; i < piv_length; i++) 
               x[i] = A[piv[i]];

            return x;
         }


      public :

         /** LU Decomposition
             @param  A   Rectangular matrix
             @return     LU Decomposition object to access L, U and piv.
         */

         LU (const Matrix<Real> &A) : LU_(A), m(A.num_rows()), n(A.num_cols()), 
                                      piv(A.num_rows())
	
         {

            // Use a "left-looking", dot-product, Crout/Doolittle algorithm.


            for (int i = 0; i < m; i++) {
               piv[i] = i;
            }
            pivsign = 1;
            Real *LUrowi = 0;;
            Vector<Real> LUcolj(m);

            // Outer loop.

            for (int j = 0; j < n; j++) {

               // Make a copy of the j-th column to localize references.

               for (int i = 0; i < m; i++) {
                  LUcolj[i] = LU_[i][j];
               }

               // Apply previous transformations.

               for (int i = 0; i < m; i++) {
                  LUrowi = LU_[i];

                  // Most of the time is spent in the following dot product.

                  int kmax = min(i,j);
                  double s = Real(0.0);
                  for (int k = 0; k < kmax; k++) {
                     s += LUrowi[k]*LUcolj[k];
                  }

                  LUrowi[j] = LUcolj[i] -= s;
               }
   
               // Find pivot and exchange if necessary.

               int p = j;
               for (int i = j+1; i < m; i++) {
                  if (abs(LUcolj[i]) > abs(LUcolj[p])) {
                     p = i;
                  }
               }
               if (p != j) {
                  int k=0;
                  for (k = 0; k < n; k++) {
                     double t = LU_[p][k]; 
                     LU_[p][k] = LU_[j][k]; 
                     LU_[j][k] = t;
                  }
                  k = piv[p]; 
                  piv[p] = piv[j]; 
                  piv[j] = k;
                  pivsign = -pivsign;
               }

               // Compute multipliers.
         
               if ((j < m) && (LU_[j][j] != Real(0.0))) {
                  for (int i = j+1; i < m; i++) {
                     LU_[i][j] /= LU_[j][j];
                  }
               }
            }
         }


         /** Is the matrix nonsingular?
             @return     1 (true)  if upper triangular factor U (and hence A) 
             is nonsingular, 0 otherwise.
         */

         int isNonsingular () {
            for (int j = 0; j < n; j++) {
               if (LU_[j][j] == 0)
                  return 0;
            }
            return 1;
         }

         /** Return lower triangular factor
             @return     L
         */

         Matrix<Real> getL () {
            Matrix<Real> L_(m,n);
            for (int i = 0; i < m; i++) {
               for (int j = 0; j < n; j++) {
                  if (i > j) {
                     L_[i][j] = LU_[i][j];
                  } else if (i == j) {
                     L_[i][j] = Real(1.0);
                  } else {
                     L_[i][j] = Real(0.0);
                  }
               }
            }
            return L_;
         }

         /** Return upper triangular factor
             @return     U portion of LU factorization.
         */

         Matrix<Real> getU () {
            Matrix<Real> U_(n,n);
            for (int i = 0; i < n; i++) {
               for (int j = 0; j < n; j++) {
                  if (i <= j) {
                     U_[i][j] = LU_[i][j];
                  } else {
                     U_[i][j] = Real(0.0);
                  }
               }
            }
            return U_;
         }

         /** Return pivot permutation vector
             @return     piv
         */

         Vector<int> getPivot () {
            return piv;
         }


         /** Compute determinant using LU factors.
             @return     determinant of A, or 0 if A is not square.
         */

         Real det () {
            if (m != n) {
               return Real(0);
            }
            Real d = Real(pivsign);
            for (int j = 0; j < n; j++) {
               d *= LU_[j][j];
            }
            return d;
         }

         /** Solve A*X = B
             @param  B   A Matrix with as many rows as A and any number of columns.
             @return     X so that L*U*X = B(piv,:), if B is nonconformant, returns
             Real(0.0) (null) array.
         */

         Matrix<Real> solve (const Matrix<Real> &B) 
         {

            /* Dimensions: A is mxn, X is nxk, B is mxk */
      
            if (B.num_rows() != m) {
               return Matrix<Real>();
            }
            if (!isNonsingular()) {
               return Matrix<Real>();
            }

            // Copy right hand side with pivoting
            int nx = B.num_cols();


            Matrix<Real> X = permute_copy(B, piv, 0, nx-1);

            // Solve L*Y = B(piv,:)
            for (int k = 0; k < n; k++) {
               for (int i = k+1; i < n; i++) {
                  for (int j = 0; j < nx; j++) {
                     X[i][j] -= X[k][j]*LU_[i][k];
                  }
               }
            }
            // Solve U*X = Y;
            for (int k = n-1; k >= 0; k--) {
               for (int j = 0; j < nx; j++) {
                  X[k][j] /= LU_[k][k];
               }
               for (int i = 0; i < k; i++) {
                  for (int j = 0; j < nx; j++) {
                     X[i][j] -= X[k][j]*LU_[i][k];
                  }
               }
            }
            return X;
         }


         /** Solve A*x = b, where x and b are vectors of length equal	
             to the number of rows in A.

             @param  b   a vector (Vector> of length equal to the first dimension
             of A.
             @return x a vector (Vector> so that L*U*x = b(piv), if B is nonconformant,
             returns Real(0.0) (null) array.
         */

         Vector<Real> solve (const Vector<Real> &b) 
         {

            /* Dimensions: A is mxn, X is nxk, B is mxk */
      
            if (b.size() != m) {
               return Vector<Real>();
            }
            if (!isNonsingular()) {
               return Vector<Real>();
            }


            Vector<Real> x = permute_copy(b, piv);

            // Solve L*Y = B(piv)
            for (int k = 0; k < n; k++) {
               for (int i = k+1; i < n; i++) {
                  x[i] -= x[k]*LU_[i][k];
               }
            }
      
            // Solve U*X = Y;
            for (int k = n-1; k >= 0; k--) {
               x[k] /= LU_[k][k];
               for (int i = 0; i < k; i++) 
                  x[i] -= x[k]*LU_[i][k];
            }
     

            return x;
         }

   }; /* class LU */

   inline void
   invertMatrix(Matrix<double> const& A, Matrix<double>& Ainv)
   {
      int const N = A.num_rows();
      Matrix<double> I(N, N, 0.0);
      for (int i = 0; i < N; ++i) I[i][i] = 1.0;
      LU<double> lu(A);
      Ainv = lu.solve(I);
   }

} // end namespace V3D

#endif
