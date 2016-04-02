// -*- C++ -*-

#ifndef V3D_LINEAR_TNT_H
#define V3D_LINEAR_TNT_H

// This is adapted from the TNT matrix and linear algebra library, which is in the public domain.

#include "Math/v3d_linear.h"
#include "Base/v3d_exception.h"

namespace V3D
{

   /** 
       <P>
       For a symmetric, positive definite matrix A, this function
       computes the Cholesky factorization, i.e. it computes a lower 
       triangular matrix L such that A = L*L'.
       If the matrix is not symmetric or positive definite, the function
       computes only a partial decomposition.  This can be tested with
       the is_spd() flag.

       <p>Typical usage looks like:
       <pre>
       Matrix<double> A(n,n);
       Matrix<double> L;

       ... 

       Cholesky<double> chol(A);

       if (chol.is_spd())
       L = chol.getL();
		
       else
       cout << "factorization was not complete.\n";

       </pre>


       <p>
       (Adapted from JAMA, a Java Matrix Library, developed by jointly 
       by the Mathworks and NIST; see  http://math.nist.gov/javanumerics/jama).

   */

   template <typename Real>
   struct Cholesky
   {
      private:
         Matrix<Real> L_;		// lower triangular factor
         int isspd;			// 1 if matrix to be factored was SPD

      public:
         Cholesky();
         Cholesky(const Matrix<Real> &A);
         Matrix<Real> const& getL() const;
         Vector<Real> solve(const Vector<Real> &B);
         Matrix<Real> solve(const Matrix<Real> &B);
         int is_spd() const;

   };

   template <class Real>
   Cholesky<Real>::Cholesky() : L_(Real(0.0)), isspd(0) {}

/**
   @return 1, if original matrix to be factored was symmetric 
   positive-definite (SPD).
*/
   template <class Real>
   int Cholesky<Real>::is_spd() const
   {
      return isspd;
   }

/**
   @return the lower triangular factor, L, such that L*L'=A.
*/
   template <class Real>
   Matrix<Real> const& Cholesky<Real>::getL() const
   {
      return L_;
   }

/**
   Constructs a lower triangular matrix L, such that L*L'= A.
   If A is not symmetric positive-definite (SPD), only a
   partial factorization is performed.  If is_spd()
   evalutate true (1) then the factorizaiton was successful.
*/
   template <class Real>
   Cholesky<Real>::Cholesky(const Matrix<Real> &A)
   {
      int m = A.num_rows();
      int n = A.num_cols();
	
      isspd = (m == n);

      if (m != n)
      {
         return;
      }

      L_ = Matrix<Real>(n,n);


      // Main loop.
      for (int j = 0; j < n; j++) 
      {
         Real d = Real(0.0);
         for (int k = 0; k < j; k++) 
         {
            Real s = Real(0.0);
            for (int i = 0; i < k; i++) 
            {
               s += L_[k][i]*L_[j][i];
            }
            L_[j][k] = s = (A[j][k] - s)/L_[k][k];
            d = d + s*s;
            isspd = isspd && (A[k][j] == A[j][k]); 
         }
         d = A[j][j] - d;
         isspd = isspd && (d > Real(0.0));
         L_[j][j] = sqrt(d > Real(0.0) ? d : Real(0.0));
         for (int k = j+1; k < n; k++) 
         {
            L_[j][k] = Real(0.0);
         }
      }
   }

/**

Solve a linear system A*x = b, using the previously computed
cholesky factorization of A: L*L'.

@param  b   matrix with as many rows as A and any number of columns.
@return     x so that L*L'*x = b.  If b is nonconformat, or if A
was not symmetric posidtive definite, a null (Real(0.0))
array is returned.
*/
   template <class Real>
   Vector<Real> Cholesky<Real>::solve(const Vector<Real> &b)
   {
      int n = L_.num_rows();
      //if (b.dim() != n) return Vector<Real>();
      if (int(b.size()) != n) return Vector<Real>();


      Vector<Real> x = b;


      // Solve L*y = b;
      for (int k = 0; k < n; k++) 
      {
         for (int i = 0; i < k; i++) 
            x[k] -= x[i]*L_[k][i];
         x[k] /= L_[k][k];
		
      }

      // Solve L'*X = Y;
      for (int k = n-1; k >= 0; k--) 
      {
         for (int i = k+1; i < n; i++) 
            x[k] -= x[i]*L_[i][k];
         x[k] /= L_[k][k];
      }

      return x;
   }


/**

Solve a linear system A*X = B, using the previously computed
cholesky factorization of A: L*L'.

@param  B   A Matrix with as many rows as A and any number of columns.
@return     X so that L*L'*X = B.  If B is nonconformat, or if A
was not symmetric posidtive definite, a null (Real(0.0))
array is returned.
*/
   template <class Real>
   Matrix<Real> Cholesky<Real>::solve(const Matrix<Real> &B)
   {
      int n = L_.num_rows();
      if (B.num_rows() != n)
         return Matrix<Real>();


      Matrix<Real> X = B;
      int nx = B.num_cols();

      // Solve L*y = b;
      for (int j=0; j< nx; j++)
      {
         for (int k = 0; k < n; k++) 
         {
            for (int i = 0; i < k; i++) 
               X[k][j] -= X[i][j]*L_[k][i];
            X[k][j] /= L_[k][k];
         }
      }

      // Solve L'*X = Y;
      for (int j=0; j<nx; j++)
      {
         for (int k = n-1; k >= 0; k--) 
         {
            for (int i = k+1; i < n; i++) 
               X[k][j] -= X[i][j]*L_[i][k];
            X[k][j] /= L_[k][k];
         }
      }

      return X;
   }

//----------------------------------------------------------------------

   /** 

   Computes eigenvalues and eigenvectors of a real (non-complex)
   matrix. 
   <P>
   If A is symmetric, then A = V*D*V' where the eigenvalue matrix D is
   diagonal and the eigenvector matrix V is orthogonal. That is,
   the diagonal values of D are the eigenvalues, and
   V*V' = I, where I is the identity matrix.  The columns of V 
   represent the eigenvectors in the sense that A*V = V*D.
    
   <P>
   If A is not symmetric, then the eigenvalue matrix D is block diagonal
   with the real eigenvalues in 1-by-1 blocks and any complex eigenvalues,
   a + i*b, in 2-by-2 blocks, [a, b; -b, a].  That is, if the complex
   eigenvalues look like
   <pre>

   u + iv     .        .          .      .    .
   .      u - iv     .          .      .    .
   .        .      a + ib       .      .    .
   .        .        .        a - ib   .    .
   .        .        .          .      x    .
   .        .        .          .      .    y
   </pre>
   then D looks like
   <pre>

   u        v        .          .      .    .
   -v        u        .          .      .    . 
   .        .        a          b      .    .
   .        .       -b          a      .    .
   .        .        .          .      x    .
   .        .        .          .      .    y
   </pre>
   This keeps V a real matrix in both symmetric and non-symmetric
   cases, and A*V = V*D.
    
    
    
   <p>
   The matrix V may be badly
   conditioned, or even singular, so the validity of the equation
   A = V*D*inverse(V) depends upon the condition number of V.

   <p>
   (Adapted from JAMA, a Java Matrix Library, developed by jointly 
   by the Mathworks and NIST (see  http://math.nist.gov/javanumerics/jama),
   which in turn, were based on original EISPACK routines.
   **/

   template <class Real>
   class Eigenvalue
   {


         /** Row and column dimension (square matrix).  */
         int n;

         int issymmetric; /* boolean*/

         /** Arrays for internal storage of eigenvalues. */

         Vector<Real> d;         /* real part */
         Vector<Real> e;         /* img part */

         /** Array for internal storage of eigenvectors. */
         Matrix<Real> V;

         /* Array for internal storage of nonsymmetric Hessenberg form.
            @serial internal storage of nonsymmetric Hessenberg form.
         */
         Matrix<Real> H;
   

         /* Working storage for nonsymmetric algorithm.
            @serial working storage for nonsymmetric algorithm.
         */
         Vector<Real> ort;


         // Symmetric Householder reduction to tridiagonal form.

         void tred2() {

            //  This is derived from the Algol procedures tred2 by
            //  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
            //  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
            //  Fortran subroutine in EISPACK.

            for (int j = 0; j < n; j++) {
               d[j] = V[n-1][j];
            }

            // Householder reduction to tridiagonal form.
   
            for (int i = n-1; i > 0; i--) {
   
               // Scale to avoid under/overflow.
   
               Real scale = Real(0.0);
               Real h = Real(0.0);
               for (int k = 0; k < i; k++) {
                  scale = scale + abs(d[k]);
               }
               if (scale == Real(0.0)) {
                  e[i] = d[i-1];
                  for (int j = 0; j < i; j++) {
                     d[j] = V[i-1][j];
                     V[i][j] = Real(0.0);
                     V[j][i] = Real(0.0);
                  }
               } else {
   
                  // Generate Householder vector.
   
                  for (int k = 0; k < i; k++) {
                     d[k] /= scale;
                     h += d[k] * d[k];
                  }
                  Real f = d[i-1];
                  Real g = sqrt(h);
                  if (f > 0) {
                     g = -g;
                  }
                  e[i] = scale * g;
                  h = h - f * g;
                  d[i-1] = f - g;
                  for (int j = 0; j < i; j++) {
                     e[j] = Real(0.0);
                  }
   
                  // Apply similarity transformation to remaining columns.
   
                  for (int j = 0; j < i; j++) {
                     f = d[j];
                     V[j][i] = f;
                     g = e[j] + V[j][j] * f;
                     for (int k = j+1; k <= i-1; k++) {
                        g += V[k][j] * d[k];
                        e[k] += V[k][j] * f;
                     }
                     e[j] = g;
                  }
                  f = Real(0.0);
                  for (int j = 0; j < i; j++) {
                     e[j] /= h;
                     f += e[j] * d[j];
                  }
                  Real hh = f / (h + h);
                  for (int j = 0; j < i; j++) {
                     e[j] -= hh * d[j];
                  }
                  for (int j = 0; j < i; j++) {
                     f = d[j];
                     g = e[j];
                     for (int k = j; k <= i-1; k++) {
                        V[k][j] -= (f * e[k] + g * d[k]);
                     }
                     d[j] = V[i-1][j];
                     V[i][j] = Real(0.0);
                  }
               }
               d[i] = h;
            }
   
            // Accumulate transformations.
   
            for (int i = 0; i < n-1; i++) {
               V[n-1][i] = V[i][i];
               V[i][i] = Real(1.0);
               Real h = d[i+1];
               if (h != Real(0.0)) {
                  for (int k = 0; k <= i; k++) {
                     d[k] = V[k][i+1] / h;
                  }
                  for (int j = 0; j <= i; j++) {
                     Real g = Real(0.0);
                     for (int k = 0; k <= i; k++) {
                        g += V[k][i+1] * V[k][j];
                     }
                     for (int k = 0; k <= i; k++) {
                        V[k][j] -= g * d[k];
                     }
                  }
               }
               for (int k = 0; k <= i; k++) {
                  V[k][i+1] = Real(0.0);
               }
            }
            for (int j = 0; j < n; j++) {
               d[j] = V[n-1][j];
               V[n-1][j] = Real(0.0);
            }
            V[n-1][n-1] = Real(1.0);
            e[0] = Real(0.0);
         } 

         // Symmetric tridiagonal QL algorithm.
   
         void tql2 () {

            //  This is derived from the Algol procedures tql2, by
            //  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
            //  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
            //  Fortran subroutine in EISPACK.
   
            for (int i = 1; i < n; i++) {
               e[i-1] = e[i];
            }
            e[n-1] = Real(0.0);
   
            Real f = Real(0.0);
            Real tst1 = Real(0.0);
            Real eps = pow(2.0,-52.0);
            for (int l = 0; l < n; l++) {

               // Find small subdiagonal element
   
               tst1 = max(tst1,abs(d[l]) + abs(e[l]));
               int m = l;

               // Original while-loop from Java code
               while (m < n) {
                  if (abs(e[m]) <= eps*tst1) {
                     break;
                  }
                  m++;
               }

   
               // If m == l, d[l] is an eigenvalue,
               // otherwise, iterate.
   
               if (m > l) {
                  int iter = 0;
                  do {
                     iter = iter + 1;  // (Could check iteration count here.)
   
                     // Compute implicit shift
   
                     Real g = d[l];
                     Real p = (d[l+1] - g) / (2.0 * e[l]);
                     Real r = hypot(p, static_cast<Real>(Real(1.0)));
                     if (p < 0) {
                        r = -r;
                     }
                     d[l] = e[l] / (p + r);
                     d[l+1] = e[l] * (p + r);
                     Real dl1 = d[l+1];
                     Real h = g - d[l];
                     for (int i = l+2; i < n; i++) {
                        d[i] -= h;
                     }
                     f = f + h;
   
                     // Implicit QL transformation.
   
                     p = d[m];
                     Real c = Real(1.0);
                     Real c2 = c;
                     Real c3 = c;
                     Real el1 = e[l+1];
                     Real s = Real(0.0);
                     Real s2 = Real(0.0);
                     for (int i = m-1; i >= l; i--) {
                        c3 = c2;
                        c2 = c;
                        s2 = s;
                        g = c * e[i];
                        h = c * p;
                        r = hypot(p,e[i]);
                        e[i+1] = s * r;
                        s = e[i] / r;
                        c = p / r;
                        p = c * d[i] - s * g;
                        d[i+1] = h + s * (c * g + s * d[i]);
   
                        // Accumulate transformation.
   
                        for (int k = 0; k < n; k++) {
                           h = V[k][i+1];
                           V[k][i+1] = s * V[k][i] + c * h;
                           V[k][i] = c * V[k][i] - s * h;
                        }
                     }
                     p = -s * s2 * c3 * el1 * e[l] / dl1;
                     e[l] = s * p;
                     d[l] = c * p;
   
                     // Check for convergence.
   
                  } while (abs(e[l]) > eps*tst1);
               }
               d[l] = d[l] + f;
               e[l] = Real(0.0);
            }
     
            // Sort eigenvalues and corresponding vectors.
   
            for (int i = 0; i < n-1; i++) {
               int k = i;
               Real p = d[i];
               for (int j = i+1; j < n; j++) {
                  if (d[j] < p) {
                     k = j;
                     p = d[j];
                  }
               }
               if (k != i) {
                  d[k] = d[i];
                  d[i] = p;
                  for (int j = 0; j < n; j++) {
                     p = V[j][i];
                     V[j][i] = V[j][k];
                     V[j][k] = p;
                  }
               }
            }
         }

         // Nonsymmetric reduction to Hessenberg form.

         void orthes () {
   
            //  This is derived from the Algol procedures orthes and ortran,
            //  by Martin and Wilkinson, Handbook for Auto. Comp.,
            //  Vol.ii-Linear Algebra, and the corresponding
            //  Fortran subroutines in EISPACK.
   
            int low = 0;
            int high = n-1;
   
            for (int m = low+1; m <= high-1; m++) {
   
               // Scale column.
   
               Real scale = Real(0.0);
               for (int i = m; i <= high; i++) {
                  scale = scale + abs(H[i][m-1]);
               }
               if (scale != Real(0.0)) {
   
                  // Compute Householder transformation.
   
                  Real h = Real(0.0);
                  for (int i = high; i >= m; i--) {
                     ort[i] = H[i][m-1]/scale;
                     h += ort[i] * ort[i];
                  }
                  Real g = sqrt(h);
                  if (ort[m] > 0) {
                     g = -g;
                  }
                  h = h - ort[m] * g;
                  ort[m] = ort[m] - g;
   
                  // Apply Householder similarity transformation
                  // H = (I-u*u'/h)*H*(I-u*u')/h)
   
                  for (int j = m; j < n; j++) {
                     Real f = Real(0.0);
                     for (int i = high; i >= m; i--) {
                        f += ort[i]*H[i][j];
                     }
                     f = f/h;
                     for (int i = m; i <= high; i++) {
                        H[i][j] -= f*ort[i];
                     }
                  }
   
                  for (int i = 0; i <= high; i++) {
                     Real f = Real(0.0);
                     for (int j = high; j >= m; j--) {
                        f += ort[j]*H[i][j];
                     }
                     f = f/h;
                     for (int j = m; j <= high; j++) {
                        H[i][j] -= f*ort[j];
                     }
                  }
                  ort[m] = scale*ort[m];
                  H[m][m-1] = scale*g;
               }
            }
   
            // Accumulate transformations (Algol's ortran).

            for (int i = 0; i < n; i++) {
               for (int j = 0; j < n; j++) {
                  V[i][j] = (i == j ? Real(1.0) : Real(0.0));
               }
            }

            for (int m = high-1; m >= low+1; m--) {
               if (H[m][m-1] != Real(0.0)) {
                  for (int i = m+1; i <= high; i++) {
                     ort[i] = H[i][m-1];
                  }
                  for (int j = m; j <= high; j++) {
                     Real g = Real(0.0);
                     for (int i = m; i <= high; i++) {
                        g += ort[i] * V[i][j];
                     }
                     // Double division avoids possible underflow
                     g = (g / ort[m]) / H[m][m-1];
                     for (int i = m; i <= high; i++) {
                        V[i][j] += g * ort[i];
                     }
                  }
               }
            }
         }


         // Complex scalar division.

         Real cdivr, cdivi;
         void cdiv(Real xr, Real xi, Real yr, Real yi) {
            Real r,d;
            if (abs(yr) > abs(yi)) {
               r = yi/yr;
               d = yr + r*yi;
               cdivr = (xr + r*xi)/d;
               cdivi = (xi - r*xr)/d;
            } else {
               r = yr/yi;
               d = yi + r*yr;
               cdivr = (r*xr + xi)/d;
               cdivi = (r*xi - xr)/d;
            }
         }


         // Nonsymmetric reduction from Hessenberg to real Schur form.

         void hqr2 () {
   
            //  This is derived from the Algol procedure hqr2,
            //  by Martin and Wilkinson, Handbook for Auto. Comp.,
            //  Vol.ii-Linear Algebra, and the corresponding
            //  Fortran subroutine in EISPACK.
   
            // Initialize
   
            int nn = this->n;
            int n = nn-1;
            int low = 0;
            int high = nn-1;
            Real eps = pow(2.0,-52.0);
            Real exshift = Real(0.0);
            Real p=0,q=0,r=0,s=0,z=0,t,w,x,y;
   
            // Store roots isolated by balanc and compute matrix norm
   
            Real norm = Real(0.0);
            for (int i = 0; i < nn; i++) {
               if ((i < low) || (i > high)) {
                  d[i] = H[i][i];
                  e[i] = Real(0.0);
               }
               for (int j = max(i-1,0); j < nn; j++) {
                  norm = norm + abs(H[i][j]);
               }
            }
   
            // Outer loop over eigenvalue index
   
            int iter = 0;
            while (n >= low) {
   
               // Look for single small sub-diagonal element
   
               int l = n;
               while (l > low) {
                  s = abs(H[l-1][l-1]) + abs(H[l][l]);
                  if (s == Real(0.0)) {
                     s = norm;
                  }
                  if (abs(H[l][l-1]) < eps * s) {
                     break;
                  }
                  l--;
               }
       
               // Check for convergence
               // One root found
   
               if (l == n) {
                  H[n][n] = H[n][n] + exshift;
                  d[n] = H[n][n];
                  e[n] = Real(0.0);
                  n--;
                  iter = 0;
   
                  // Two roots found
   
               } else if (l == n-1) {
                  w = H[n][n-1] * H[n-1][n];
                  p = (H[n-1][n-1] - H[n][n]) / 2.0;
                  q = p * p + w;
                  z = sqrt(abs(q));
                  H[n][n] = H[n][n] + exshift;
                  H[n-1][n-1] = H[n-1][n-1] + exshift;
                  x = H[n][n];
   
                  // Real pair
   
                  if (q >= 0) {
                     if (p >= 0) {
                        z = p + z;
                     } else {
                        z = p - z;
                     }
                     d[n-1] = x + z;
                     d[n] = d[n-1];
                     if (z != Real(0.0)) {
                        d[n] = x - w / z;
                     }
                     e[n-1] = Real(0.0);
                     e[n] = Real(0.0);
                     x = H[n][n-1];
                     s = abs(x) + abs(z);
                     p = x / s;
                     q = z / s;
                     r = sqrt(p * p+q * q);
                     p = p / r;
                     q = q / r;
   
                     // Row modification
   
                     for (int j = n-1; j < nn; j++) {
                        z = H[n-1][j];
                        H[n-1][j] = q * z + p * H[n][j];
                        H[n][j] = q * H[n][j] - p * z;
                     }
   
                     // Column modification
   
                     for (int i = 0; i <= n; i++) {
                        z = H[i][n-1];
                        H[i][n-1] = q * z + p * H[i][n];
                        H[i][n] = q * H[i][n] - p * z;
                     }
   
                     // Accumulate transformations
   
                     for (int i = low; i <= high; i++) {
                        z = V[i][n-1];
                        V[i][n-1] = q * z + p * V[i][n];
                        V[i][n] = q * V[i][n] - p * z;
                     }
   
                     // Complex pair
   
                  } else {
                     d[n-1] = x + p;
                     d[n] = x + p;
                     e[n-1] = z;
                     e[n] = -z;
                  }
                  n = n - 2;
                  iter = 0;
   
                  // No convergence yet
   
               } else {
   
                  // Form shift
   
                  x = H[n][n];
                  y = Real(0.0);
                  w = Real(0.0);
                  if (l < n) {
                     y = H[n-1][n-1];
                     w = H[n][n-1] * H[n-1][n];
                  }
   
                  // Wilkinson's original ad hoc shift
   
                  if (iter == 10) {
                     exshift += x;
                     for (int i = low; i <= n; i++) {
                        H[i][i] -= x;
                     }
                     s = abs(H[n][n-1]) + abs(H[n-1][n-2]);
                     x = y = 0.75 * s;
                     w = -0.4375 * s * s;
                  }

                  // MATLAB's new ad hoc shift

                  if (iter == 30) {
                     s = (y - x) / 2.0;
                     s = s * s + w;
                     if (s > 0) {
                        s = sqrt(s);
                        if (y < x) {
                           s = -s;
                        }
                        s = x - w / ((y - x) / 2.0 + s);
                        for (int i = low; i <= n; i++) {
                           H[i][i] -= s;
                        }
                        exshift += s;
                        x = y = w = 0.964;
                     }
                  }
   
                  iter = iter + 1;   // (Could check iteration count here.)
   
                  // Look for two consecutive small sub-diagonal elements
   
                  int m = n-2;
                  while (m >= l) {
                     z = H[m][m];
                     r = x - z;
                     s = y - z;
                     p = (r * s - w) / H[m+1][m] + H[m][m+1];
                     q = H[m+1][m+1] - z - r - s;
                     r = H[m+2][m+1];
                     s = abs(p) + abs(q) + abs(r);
                     p = p / s;
                     q = q / s;
                     r = r / s;
                     if (m == l) {
                        break;
                     }
                     if (abs(H[m][m-1]) * (abs(q) + abs(r)) <
                         eps * (abs(p) * (abs(H[m-1][m-1]) + abs(z) +
                                          abs(H[m+1][m+1])))) {
                        break;
                     }
                     m--;
                  }
   
                  for (int i = m+2; i <= n; i++) {
                     H[i][i-2] = Real(0.0);
                     if (i > m+2) {
                        H[i][i-3] = Real(0.0);
                     }
                  }
   
                  // Double QR step involving rows l:n and columns m:n
   
                  for (int k = m; k <= n-1; k++) {
                     int notlast = (k != n-1);
                     if (k != m) {
                        p = H[k][k-1];
                        q = H[k+1][k-1];
                        r = (notlast ? H[k+2][k-1] : Real(0.0));
                        x = abs(p) + abs(q) + abs(r);
                        if (x != Real(0.0)) {
                           p = p / x;
                           q = q / x;
                           r = r / x;
                        }
                     }
                     if (x == Real(0.0)) {
                        break;
                     }
                     s = sqrt(p * p + q * q + r * r);
                     if (p < 0) {
                        s = -s;
                     }
                     if (s != 0) {
                        if (k != m) {
                           H[k][k-1] = -s * x;
                        } else if (l != m) {
                           H[k][k-1] = -H[k][k-1];
                        }
                        p = p + s;
                        x = p / s;
                        y = q / s;
                        z = r / s;
                        q = q / p;
                        r = r / p;
   
                        // Row modification
   
                        for (int j = k; j < nn; j++) {
                           p = H[k][j] + q * H[k+1][j];
                           if (notlast) {
                              p = p + r * H[k+2][j];
                              H[k+2][j] = H[k+2][j] - p * z;
                           }
                           H[k][j] = H[k][j] - p * x;
                           H[k+1][j] = H[k+1][j] - p * y;
                        }
   
                        // Column modification
   
                        for (int i = 0; i <= min(n,k+3); i++) {
                           p = x * H[i][k] + y * H[i][k+1];
                           if (notlast) {
                              p = p + z * H[i][k+2];
                              H[i][k+2] = H[i][k+2] - p * r;
                           }
                           H[i][k] = H[i][k] - p;
                           H[i][k+1] = H[i][k+1] - p * q;
                        }
   
                        // Accumulate transformations
   
                        for (int i = low; i <= high; i++) {
                           p = x * V[i][k] + y * V[i][k+1];
                           if (notlast) {
                              p = p + z * V[i][k+2];
                              V[i][k+2] = V[i][k+2] - p * r;
                           }
                           V[i][k] = V[i][k] - p;
                           V[i][k+1] = V[i][k+1] - p * q;
                        }
                     }  // (s != 0)
                  }  // k loop
               }  // check convergence
            }  // while (n >= low)
      
            // Backsubstitute to find vectors of upper triangular form

            if (norm == Real(0.0)) {
               return;
            }
   
            for (n = nn-1; n >= 0; n--) {
               p = d[n];
               q = e[n];
   
               // Real vector
   
               if (q == 0) {
                  int l = n;
                  H[n][n] = Real(1.0);
                  for (int i = n-1; i >= 0; i--) {
                     w = H[i][i] - p;
                     r = Real(0.0);
                     for (int j = l; j <= n; j++) {
                        r = r + H[i][j] * H[j][n];
                     }
                     if (e[i] < Real(0.0)) {
                        z = w;
                        s = r;
                     } else {
                        l = i;
                        if (e[i] == Real(0.0)) {
                           if (w != Real(0.0)) {
                              H[i][n] = -r / w;
                           } else {
                              H[i][n] = -r / (eps * norm);
                           }
   
                           // Solve real equations
   
                        } else {
                           x = H[i][i+1];
                           y = H[i+1][i];
                           q = (d[i] - p) * (d[i] - p) + e[i] * e[i];
                           t = (x * s - z * r) / q;
                           H[i][n] = t;
                           if (abs(x) > abs(z)) {
                              H[i+1][n] = (-r - w * t) / x;
                           } else {
                              H[i+1][n] = (-s - y * t) / z;
                           }
                        }
   
                        // Overflow control
   
                        t = abs(H[i][n]);
                        if ((eps * t) * t > 1) {
                           for (int j = i; j <= n; j++) {
                              H[j][n] = H[j][n] / t;
                           }
                        }
                     }
                  }
   
                  // Complex vector
   
               } else if (q < 0) {
                  int l = n-1;

                  // Last vector component imaginary so matrix is triangular
   
                  if (abs(H[n][n-1]) > abs(H[n-1][n])) {
                     H[n-1][n-1] = q / H[n][n-1];
                     H[n-1][n] = -(H[n][n] - p) / H[n][n-1];
                  } else {
                     cdiv(Real(0.0),-H[n-1][n],H[n-1][n-1]-p,q);
                     H[n-1][n-1] = cdivr;
                     H[n-1][n] = cdivi;
                  }
                  H[n][n-1] = Real(0.0);
                  H[n][n] = Real(1.0);
                  for (int i = n-2; i >= 0; i--) {
                     Real ra,sa,vr,vi;
                     ra = Real(0.0);
                     sa = Real(0.0);
                     for (int j = l; j <= n; j++) {
                        ra = ra + H[i][j] * H[j][n-1];
                        sa = sa + H[i][j] * H[j][n];
                     }
                     w = H[i][i] - p;
   
                     if (e[i] < Real(0.0)) {
                        z = w;
                        r = ra;
                        s = sa;
                     } else {
                        l = i;
                        if (e[i] == 0) {
                           cdiv(-ra,-sa,w,q);
                           H[i][n-1] = cdivr;
                           H[i][n] = cdivi;
                        } else {
   
                           // Solve complex equations
   
                           x = H[i][i+1];
                           y = H[i+1][i];
                           vr = (d[i] - p) * (d[i] - p) + e[i] * e[i] - q * q;
                           vi = (d[i] - p) * 2.0 * q;
                           if ((vr == Real(0.0)) && (vi == Real(0.0))) {
                              vr = eps * norm * (abs(w) + abs(q) +
                                                 abs(x) + abs(y) + abs(z));
                           }
                           cdiv(x*r-z*ra+q*sa,x*s-z*sa-q*ra,vr,vi);
                           H[i][n-1] = cdivr;
                           H[i][n] = cdivi;
                           if (abs(x) > (abs(z) + abs(q))) {
                              H[i+1][n-1] = (-ra - w * H[i][n-1] + q * H[i][n]) / x;
                              H[i+1][n] = (-sa - w * H[i][n] - q * H[i][n-1]) / x;
                           } else {
                              cdiv(-r-y*H[i][n-1],-s-y*H[i][n],z,q);
                              H[i+1][n-1] = cdivr;
                              H[i+1][n] = cdivi;
                           }
                        }
   
                        // Overflow control

                        t = max(abs(H[i][n-1]),abs(H[i][n]));
                        if ((eps * t) * t > 1) {
                           for (int j = i; j <= n; j++) {
                              H[j][n-1] = H[j][n-1] / t;
                              H[j][n] = H[j][n] / t;
                           }
                        }
                     }
                  }
               }
            }
   
            // Vectors of isolated roots
   
            for (int i = 0; i < nn; i++) {
               if (i < low || i > high) {
                  for (int j = i; j < nn; j++) {
                     V[i][j] = H[i][j];
                  }
               }
            }
   
            // Back transformation to get eigenvectors of original matrix
   
            for (int j = nn-1; j >= low; j--) {
               for (int i = low; i <= high; i++) {
                  z = Real(0.0);
                  for (int k = low; k <= min(j,high); k++) {
                     z = z + V[i][k] * H[k][j];
                  }
                  V[i][j] = z;
               }
            }
         }

      public:


         /** Check for symmetry, then construct the eigenvalue decomposition
             @param A    Square real (non-complex) matrix
         */

         Eigenvalue(const Matrix<Real> &A) {
            n = A.num_cols();
            V = Matrix<Real>(n,n);
            d = Vector<Real>(n);
            e = Vector<Real>(n);

            issymmetric = 1;
            for (int j = 0; (j < n) && issymmetric; j++) {
               for (int i = 0; (i < n) && issymmetric; i++) {
                  issymmetric = (A[i][j] == A[j][i]);
               }
            }

            if (issymmetric) {
               for (int i = 0; i < n; i++) {
                  for (int j = 0; j < n; j++) {
                     V[i][j] = A[i][j];
                  }
               }
   
               // Tridiagonalize.
               tred2();
   
               // Diagonalize.
               tql2();

            } else {
               H = Matrix<Real>(n,n);
               ort = Vector<Real>(n);
         
               for (int j = 0; j < n; j++) {
                  for (int i = 0; i < n; i++) {
                     H[i][j] = A[i][j];
                  }
               }
   
               // Reduce to Hessenberg form.
               orthes();
   
               // Reduce Hessenberg to real Schur form.
               hqr2();
            }
         }


         /** Return the eigenvector matrix
             @return     V
         */

         Matrix<Real> const& getV() const { return this->V; }

         void getV (Matrix<Real> &V_) {
            V_ = V;
            return;
         }

         /** Return the real parts of the eigenvalues
             @return     real(diag(D))
         */

         void getRealEigenvalues (Vector<Real> &d_) {
            d_ = d;
            return ;
         }

         /** Return the imaginary parts of the eigenvalues
             in parameter e_.

             @param e_: new matrix with imaginary parts of the eigenvalues.
         */
         void getImagEigenvalues (Vector<Real> &e_) {
            e_ = e;
            return;
         }

   
/** 
    Computes the block diagonal eigenvalue matrix.
    If the original matrix A is not symmetric, then the eigenvalue 
    matrix D is block diagonal with the real eigenvalues in 1-by-1 
    blocks and any complex eigenvalues,
    a + i*b, in 2-by-2 blocks, [a, b; -b, a].  That is, if the complex
    eigenvalues look like
    <pre>

    u + iv     .        .          .      .    .
    .      u - iv     .          .      .    .
    .        .      a + ib       .      .    .
    .        .        .        a - ib   .    .
    .        .        .          .      x    .
    .        .        .          .      .    y
    </pre>
    then D looks like
    <pre>

    u        v        .          .      .    .
    -v        u        .          .      .    . 
    .        .        a          b      .    .
    .        .       -b          a      .    .
    .        .        .          .      x    .
    .        .        .          .      .    y
    </pre>
    This keeps V a real matrix in both symmetric and non-symmetric
    cases, and A*V = V*D.

    @param D: upon return, the matrix is filled with the block diagonal 
    eigenvalue matrix.
	
*/
         void getD (Matrix<Real> &D) {
            D = Matrix<Real>(n,n);
            for (int i = 0; i < n; i++) {
               for (int j = 0; j < n; j++) {
                  D[i][j] = Real(0.0);
               }
               D[i][i] = d[i];
               if (e[i] > 0) {
                  D[i][i+1] = e[i];
               } else if (e[i] < 0) {
                  D[i][i-1] = e[i];
               }
            }
         }
   }; // end struct Eigenvalue

//----------------------------------------------------------------------

//    /** LU Decomposition.
//        <P>
//        For an m-by-n matrix A with m >= n, the LU decomposition is an m-by-n
//        unit lower triangular matrix L, an n-by-n upper triangular matrix U,
//        and a permutation vector piv of length m so that A(piv,:) = L*U.
//        If m < n, then L is m-by-m and U is m-by-n.
//        <P>
//        The LU decompostion with pivoting always exists, even if the matrix is
//        singular, so the constructor will never fail.  The primary use of the
//        LU decomposition is in the solution of square systems of simultaneous
//        linear equations.  This will fail if isNonsingular() returns false.
//    */
//    template <class Real>
//    class LU
//    {

//       private:

//          /* Array for internal storage of decomposition.  */
//          Matrix<Real>  LU_;
//          int m, n, pivsign; 
//          Vector<int> piv;


//          Matrix<Real> permute_copy( const Matrix<Real> &A, 
//                                     const Vector<int> &piv, int j0, int j1)
//          {
//             int piv_length = piv.size();

//             Matrix<Real> X(piv_length, j1-j0+1);


//             for (int i = 0; i < piv_length; i++) 
//                for (int j = j0; j <= j1; j++) 
//                   X[i][j-j0] = A[piv[i]][j];

//             return X;
//          }

//          Vector<Real> permute_copy( const Vector<Real> &A, 
//                                     const Vector<int> &piv)
//          {
//             int piv_length = piv.size();
//             if (piv_length != A.dim())
//                return Vector<Real>();

//             Vector<Real> x(piv_length);


//             for (int i = 0; i < piv_length; i++) 
//                x[i] = A[piv[i]];

//             return x;
//          }


//       public :

//          /** LU Decomposition
//              @param  A   Rectangular matrix
//              @return     LU Decomposition object to access L, U and piv.
//          */

//          LU (const Matrix<Real> &A) : LU_(A), m(A.num_rows()), n(A.num_cols()), 
//                                       piv(A.num_rows())
	
//          {

//             // Use a "left-looking", dot-product, Crout/Doolittle algorithm.


//             for (int i = 0; i < m; i++) {
//                piv[i] = i;
//             }
//             pivsign = 1;
//             Real *LUrowi = 0;;
//             Vector<Real> LUcolj(m);

//             // Outer loop.

//             for (int j = 0; j < n; j++) {

//                // Make a copy of the j-th column to localize references.

//                for (int i = 0; i < m; i++) {
//                   LUcolj[i] = LU_[i][j];
//                }

//                // Apply previous transformations.

//                for (int i = 0; i < m; i++) {
//                   LUrowi = LU_[i];

//                   // Most of the time is spent in the following dot product.

//                   int kmax = min(i,j);
//                   double s = Real(0.0);
//                   for (int k = 0; k < kmax; k++) {
//                      s += LUrowi[k]*LUcolj[k];
//                   }

//                   LUrowi[j] = LUcolj[i] -= s;
//                }
   
//                // Find pivot and exchange if necessary.

//                int p = j;
//                for (int i = j+1; i < m; i++) {
//                   if (abs(LUcolj[i]) > abs(LUcolj[p])) {
//                      p = i;
//                   }
//                }
//                if (p != j) {
//                   int k=0;
//                   for (k = 0; k < n; k++) {
//                      double t = LU_[p][k]; 
//                      LU_[p][k] = LU_[j][k]; 
//                      LU_[j][k] = t;
//                   }
//                   k = piv[p]; 
//                   piv[p] = piv[j]; 
//                   piv[j] = k;
//                   pivsign = -pivsign;
//                }

//                // Compute multipliers.
         
//                if ((j < m) && (LU_[j][j] != Real(0.0))) {
//                   for (int i = j+1; i < m; i++) {
//                      LU_[i][j] /= LU_[j][j];
//                   }
//                }
//             }
//          }


//          /** Is the matrix nonsingular?
//              @return     1 (true)  if upper triangular factor U (and hence A) 
//              is nonsingular, 0 otherwise.
//          */

//          int isNonsingular () {
//             for (int j = 0; j < n; j++) {
//                if (LU_[j][j] == 0)
//                   return 0;
//             }
//             return 1;
//          }

//          /** Return lower triangular factor
//              @return     L
//          */

//          Matrix<Real> getL () {
//             Matrix<Real> L_(m,n);
//             for (int i = 0; i < m; i++) {
//                for (int j = 0; j < n; j++) {
//                   if (i > j) {
//                      L_[i][j] = LU_[i][j];
//                   } else if (i == j) {
//                      L_[i][j] = Real(1.0);
//                   } else {
//                      L_[i][j] = Real(0.0);
//                   }
//                }
//             }
//             return L_;
//          }

//          /** Return upper triangular factor
//              @return     U portion of LU factorization.
//          */

//          Matrix<Real> getU () {
//             Matrix<Real> U_(n,n);
//             for (int i = 0; i < n; i++) {
//                for (int j = 0; j < n; j++) {
//                   if (i <= j) {
//                      U_[i][j] = LU_[i][j];
//                   } else {
//                      U_[i][j] = Real(0.0);
//                   }
//                }
//             }
//             return U_;
//          }

//          /** Return pivot permutation vector
//              @return     piv
//          */

//          Vector<int> getPivot () {
//             return piv;
//          }


//          /** Compute determinant using LU factors.
//              @return     determinant of A, or 0 if A is not square.
//          */

//          Real det () {
//             if (m != n) {
//                return Real(0);
//             }
//             Real d = Real(pivsign);
//             for (int j = 0; j < n; j++) {
//                d *= LU_[j][j];
//             }
//             return d;
//          }

//          /** Solve A*X = B
//              @param  B   A Matrix with as many rows as A and any number of columns.
//              @return     X so that L*U*X = B(piv,:), if B is nonconformant, returns
//              Real(0.0) (null) array.
//          */

//          Matrix<Real> solve (const Matrix<Real> &B) 
//          {

//             /* Dimensions: A is mxn, X is nxk, B is mxk */
      
//             if (B.num_rows() != m) {
//                return Matrix<Real>();
//             }
//             if (!isNonsingular()) {
//                return Matrix<Real>();
//             }

//             // Copy right hand side with pivoting
//             int nx = B.num_cols();


//             Matrix<Real> X = permute_copy(B, piv, 0, nx-1);

//             // Solve L*Y = B(piv,:)
//             for (int k = 0; k < n; k++) {
//                for (int i = k+1; i < n; i++) {
//                   for (int j = 0; j < nx; j++) {
//                      X[i][j] -= X[k][j]*LU_[i][k];
//                   }
//                }
//             }
//             // Solve U*X = Y;
//             for (int k = n-1; k >= 0; k--) {
//                for (int j = 0; j < nx; j++) {
//                   X[k][j] /= LU_[k][k];
//                }
//                for (int i = 0; i < k; i++) {
//                   for (int j = 0; j < nx; j++) {
//                      X[i][j] -= X[k][j]*LU_[i][k];
//                   }
//                }
//             }
//             return X;
//          }


//          /** Solve A*x = b, where x and b are vectors of length equal	
//              to the number of rows in A.

//              @param  b   a vector (Vector> of length equal to the first dimension
//              of A.
//              @return x a vector (Vector> so that L*U*x = b(piv), if B is nonconformant,
//              returns Real(0.0) (null) array.
//          */

//          Vector<Real> solve (const Vector<Real> &b) 
//          {

//             /* Dimensions: A is mxn, X is nxk, B is mxk */
      
//             if (b.dim() != m) {
//                return Vector<Real>();
//             }
//             if (!isNonsingular()) {
//                return Vector<Real>();
//             }


//             Vector<Real> x = permute_copy(b, piv);

//             // Solve L*Y = B(piv)
//             for (int k = 0; k < n; k++) {
//                for (int i = k+1; i < n; i++) {
//                   x[i] -= x[k]*LU_[i][k];
//                }
//             }
      
//             // Solve U*X = Y;
//             for (int k = n-1; k >= 0; k--) {
//                x[k] /= LU_[k][k];
//                for (int i = 0; i < k; i++) 
//                   x[i] -= x[k]*LU_[i][k];
//             }
     

//             return x;
//          }

//    }; /* class LU */

//----------------------------------------------------------------------

   /** 
       <p>
       Classical QR Decompisition:
       for an m-by-n matrix A with m >= n, the QR decomposition is an m-by-n
       orthogonal matrix Q and an n-by-n upper triangular matrix R so that
       A = Q*R.
       <P>
       The QR decompostion always exists, even if the matrix does not have
       full rank, so the constructor will never fail.  The primary use of the
       QR decomposition is in the least squares solution of nonsquare systems
       of simultaneous linear equations.  This will fail if isFullRank()
       returns 0 (false).

       <p>
       The Q and R factors can be retrived via the getQ() and getR()
       methods. Furthermore, a solve() method is provided to find the
       least squares solution of Ax=b using the QR factors.  

       <p>
       (Adapted from JAMA, a Java Matrix Library, developed by jointly 
       by the Mathworks and NIST; see  http://math.nist.gov/javanumerics/jama).
   */

   template <class Real>
   class QR {


         /* Array for internal storage of decomposition.
            @serial internal array storage.
         */
   
         Matrix<Real> QR_;

         /* Row and column dimensions.
            @serial column dimension.
            @serial row dimension.
         */
         int m, n;

         /* Array for internal storage of diagonal of R.
            @serial diagonal of R.
         */
         Vector<Real> Rdiag;


      public:
	
         /**
            Create a QR factorization object for A.

            @param A rectangular (m>=n) matrix.
         */
         QR(const Matrix<Real> &A)		/* constructor */
         {
            QR_ = A;
            m = A.num_rows();
            n = A.num_cols();
            Rdiag = Vector<Real>(n);
            int i=0, j=0, k=0;

            // Main loop.
            for (k = 0; k < n; k++) {
               // Compute 2-norm of k-th column without under/overflow.
               Real nrm = 0;
               for (i = k; i < m; i++) {
                  nrm = hypot(nrm,QR_[i][k]);
               }

               if (nrm != Real(0.0)) {
                  // Form k-th Householder vector.
                  if (QR_[k][k] < 0) {
                     nrm = -nrm;
                  }
                  for (i = k; i < m; i++) {
                     QR_[i][k] /= nrm;
                  }
                  QR_[k][k] += Real(1.0);

                  // Apply transformation to remaining columns.
                  for (j = k+1; j < n; j++) {
                     Real s = Real(0.0); 
                     for (i = k; i < m; i++) {
                        s += QR_[i][k]*QR_[i][j];
                     }
                     s = -s/QR_[k][k];
                     for (i = k; i < m; i++) {
                        QR_[i][j] += s*QR_[i][k];
                     }
                  }
               }
               Rdiag[k] = -nrm;
            }
         }


         /**
            Flag to denote the matrix is of full rank.

            @return 1 if matrix is full rank, 0 otherwise.
         */
         int isFullRank() const		
         {
            for (int j = 0; j < n; j++) 
            {
               if (Rdiag[j] == 0)
                  return 0;
            }
            return 1;
         }

         /** 
             Retreive the Householder vectors from QR factorization

             @returns lower trapezoidal matrix whose columns define the reflections
         */

         Matrix<Real> getHouseholder (void)  const
         {
            Matrix<Real> H(m,n);

            /* note: H is completely filled in by algorithm, so
               initializaiton of H is not necessary.
            */
            for (int i = 0; i < m; i++) 
            {
               for (int j = 0; j < n; j++) 
               {
                  if (i >= j) {
                     H[i][j] = QR_[i][j];
                  } else {
                     H[i][j] = Real(0.0);
                  }
               }
            }
            return H;
         }



         /** Return the upper triangular factor, R, of the QR factorization
             @return     R
         */

         Matrix<Real> getR() const
         {
            Matrix<Real> R(n,n);
            for (int i = 0; i < n; i++) {
               for (int j = 0; j < n; j++) {
                  if (i < j) {
                     R[i][j] = QR_[i][j];
                  } else if (i == j) {
                     R[i][j] = Rdiag[i];
                  } else {
                     R[i][j] = Real(0.0);
                  }
               }
            }
            return R;
         }
	
         /** 
             @return     Q the (ecnomy-sized) orthogonal factor (Q*R=A).
         */

         Matrix<Real> getQ() const
         {
            int i=0, j=0, k=0;

            Matrix<Real> Q(m,n);
            for (k = n-1; k >= 0; k--) {
               for (i = 0; i < m; i++) {
                  Q[i][k] = Real(0.0);
               }
               Q[k][k] = Real(1.0);
               for (j = k; j < n; j++) {
                  if (QR_[k][k] != 0) {
                     Real s = Real(0.0);
                     for (i = k; i < m; i++) {
                        s += QR_[i][k]*Q[i][j];
                     }
                     s = -s/QR_[k][k];
                     for (i = k; i < m; i++) {
                        Q[i][j] += s*QR_[i][k];
                     }
                  }
               }
            }
            return Q;
         }


         /** Least squares solution of A*x = b
             @param b     right hand side  (m-length vector).
             @return x    n-length vector that minimizes the two norm of Q*R*X-B.
             If B is non-conformant, or if QR.isFullRank() is false,
             the routine returns a null (0-length) vector.
         */

         Vector<Real> solve(const Vector<Real> &b) const
         {
//             if (b.dim() != m)		/* arrays must be conformant */
//                return Vector<Real>();
            if (b.size() != m)		/* arrays must be conformant */
               return Vector<Real>();

            if ( !isFullRank() )		/* matrix is rank deficient */
            {
               return Vector<Real>();
            }

            Vector<Real> x = b;

            // Compute Y = transpose(Q)*b
            for (int k = 0; k < n; k++) 
            {
               Real s = Real(0.0); 
               for (int i = k; i < m; i++) 
               {
                  s += QR_[i][k]*x[i];
               }
               s = -s/QR_[k][k];
               for (int i = k; i < m; i++) 
               {
                  x[i] += s*QR_[i][k];
               }
            }
            // Solve R*X = Y;
            for (int k = n-1; k >= 0; k--) 
            {
               x[k] /= Rdiag[k];
               for (int i = 0; i < k; i++) {
                  x[i] -= x[k]*QR_[i][k];
               }
            }


            /* return n x nx portion of X */
            Vector<Real> x_(n);
            for (int i=0; i<n; i++)
               x_[i] = x[i];

            return x_;
         }

         /** Least squares solution of A*X = B
             @param B     m x k Array (must conform).
             @return X     n x k Array that minimizes the two norm of Q*R*X-B. If
             B is non-conformant, or if QR.isFullRank() is false,
             the routine returns a null (Real(0.0)) array.
         */

         Matrix<Real> solve(const Matrix<Real> &B) const
         {
            if (B.num_rows() != m)		/* arrays must be conformant */
               return Matrix<Real>(Real(0.0));

            if ( !isFullRank() )		/* matrix is rank deficient */
            {
               return Matrix<Real>(Real(0.0));
            }

            int nx = B.num_cols(); 
            Matrix<Real> X = B;
            int i=0, j=0, k=0;

            // Compute Y = transpose(Q)*B
            for (k = 0; k < n; k++) {
               for (j = 0; j < nx; j++) {
                  Real s = Real(0.0); 
                  for (i = k; i < m; i++) {
                     s += QR_[i][k]*X[i][j];
                  }
                  s = -s/QR_[k][k];
                  for (i = k; i < m; i++) {
                     X[i][j] += s*QR_[i][k];
                  }
               }
            }
            // Solve R*X = Y;
            for (k = n-1; k >= 0; k--) {
               for (j = 0; j < nx; j++) {
                  X[k][j] /= Rdiag[k];
               }
               for (i = 0; i < k; i++) {
                  for (j = 0; j < nx; j++) {
                     X[i][j] -= X[k][j]*QR_[i][k];
                  }
               }
            }


            /* return n x nx portion of X */
            Matrix<Real> X_(n,nx);
            for (i=0; i<n; i++)
               for (j=0; j<nx; j++)
                  X_[i][j] = X[i][j];

            return X_;
         }
   }; // end QR

//----------------------------------------------------------------------

   /** Singular Value Decomposition.
       <P>
       For an m-by-n matrix A with m >= n, the singular value decomposition is
       an m-by-n orthogonal matrix U, an n-by-n diagonal matrix S, and
       an n-by-n orthogonal matrix V so that A = U*S*V'.
       <P>
       The singular values, sigma[k] = S[k][k], are ordered so that
       sigma[0] >= sigma[1] >= ... >= sigma[n-1].
       <P>
       The singular value decompostion always exists, so the constructor will
       never fail.  The matrix condition number and the effective numerical
       rank can be computed from this decomposition.

       <p>
       (Adapted from JAMA, a Java Matrix Library, developed by jointly 
       by the Mathworks and NIST; see  http://math.nist.gov/javanumerics/jama).
   */
   template <class Real>
   class SVD 
   {
         Matrix<Real> U, V;
         Vector<Real> s;
         int m, n;

      public:


         SVD (const Matrix<Real> &Arg)
         {
            m = Arg.num_rows();
            n = Arg.num_cols();

            if (n > m) throw V3D::Exception("SVD: #columns > #rows for the provided matrix. Please apply SVD on A^T.");

            int nu = min(m,n);
            s = Vector<Real>(min(m+1,n)); 
            U = Matrix<Real>(m, nu, Real(0));
            V = Matrix<Real>(n,n);
            Vector<Real> e(n);
            Vector<Real> work(m);
	    Matrix<Real> A(Arg);
            int wantu = 1;  					/* boolean */
            int wantv = 1;  					/* boolean */
            int i=0, j=0, k=0;

            // Reduce A to bidiagonal form, storing the diagonal elements
            // in s and the super-diagonal elements in e.

            int nct = min(m-1,n);
            int nrt = max(0,min(n-2,m));
            for (k = 0; k < max(nct,nrt); k++) {
               if (k < nct) {

                  // Compute the transformation for the k-th column and
                  // place the k-th diagonal in s[k].
                  // Compute 2-norm of k-th column without under/overflow.
                  s[k] = 0;
                  for (i = k; i < m; i++) {
                     s[k] = hypot(s[k],A[i][k]);
                  }
                  if (s[k] != Real(0.0)) {
                     if (A[k][k] < Real(0.0)) {
                        s[k] = -s[k];
                     }
                     for (i = k; i < m; i++) {
                        A[i][k] /= s[k];
                     }
                     A[k][k] += Real(1.0);
                  }
                  s[k] = -s[k];
               }
               for (j = k+1; j < n; j++) {
                  if ((k < nct) && (s[k] != Real(0.0)))  {

                     // Apply the transformation.

                     double t = 0;
                     for (i = k; i < m; i++) {
                        t += A[i][k]*A[i][j];
                     }
                     t = -t/A[k][k];
                     for (i = k; i < m; i++) {
                        A[i][j] += t*A[i][k];
                     }
                  }

                  // Place the k-th row of A into e for the
                  // subsequent calculation of the row transformation.

                  e[j] = A[k][j];
               }
               if (wantu & (k < nct)) {

                  // Place the transformation in U for subsequent back
                  // multiplication.

                  for (i = k; i < m; i++) {
                     U[i][k] = A[i][k];
                  }
               }
               if (k < nrt) {

                  // Compute the k-th row transformation and place the
                  // k-th super-diagonal in e[k].
                  // Compute 2-norm without under/overflow.
                  e[k] = 0;
                  for (i = k+1; i < n; i++) {
                     e[k] = hypot(e[k],e[i]);
                  }
                  if (e[k] != Real(0.0)) {
                     if (e[k+1] < Real(0.0)) {
                        e[k] = -e[k];
                     }
                     for (i = k+1; i < n; i++) {
                        e[i] /= e[k];
                     }
                     e[k+1] += Real(1.0);
                  }
                  e[k] = -e[k];
                  if ((k+1 < m) & (e[k] != Real(0.0))) {

                     // Apply the transformation.

                     for (i = k+1; i < m; i++) {
                        work[i] = Real(0.0);
                     }
                     for (j = k+1; j < n; j++) {
                        for (i = k+1; i < m; i++) {
                           work[i] += e[j]*A[i][j];
                        }
                     }
                     for (j = k+1; j < n; j++) {
                        double t = -e[j]/e[k+1];
                        for (i = k+1; i < m; i++) {
                           A[i][j] += t*work[i];
                        }
                     }
                  }
                  if (wantv) {

                     // Place the transformation in V for subsequent
                     // back multiplication.

                     for (i = k+1; i < n; i++) {
                        V[i][k] = e[i];
                     }
                  }
               }
            }

            // Set up the final bidiagonal matrix or order p.

            int p = min(n,m+1);
            if (nct < n) {
               s[nct] = A[nct][nct];
            }
            if (m < p) {
               s[p-1] = Real(0.0);
            }
            if (nrt+1 < p) {
               e[nrt] = A[nrt][p-1];
            }
            e[p-1] = Real(0.0);

            // If required, generate U.

            if (wantu) {
               for (j = nct; j < nu; j++) {
                  for (i = 0; i < m; i++) {
                     U[i][j] = Real(0.0);
                  }
                  U[j][j] = Real(1.0);
               }
               for (k = nct-1; k >= 0; k--) {
                  if (s[k] != Real(0.0)) {
                     for (j = k+1; j < nu; j++) {
                        double t = 0;
                        for (i = k; i < m; i++) {
                           t += U[i][k]*U[i][j];
                        }
                        t = -t/U[k][k];
                        for (i = k; i < m; i++) {
                           U[i][j] += t*U[i][k];
                        }
                     }
                     for (i = k; i < m; i++ ) {
                        U[i][k] = -U[i][k];
                     }
                     U[k][k] = Real(1.0) + U[k][k];
                     for (i = 0; i < k-1; i++) {
                        U[i][k] = Real(0.0);
                     }
                  } else {
                     for (i = 0; i < m; i++) {
                        U[i][k] = Real(0.0);
                     }
                     U[k][k] = Real(1.0);
                  }
               }
            }

            // If required, generate V.

            if (wantv) {
               for (k = n-1; k >= 0; k--) {
                  if ((k < nrt) & (e[k] != Real(0.0))) {
                     for (j = k+1; j < nu; j++) {
                        double t = 0;
                        for (i = k+1; i < n; i++) {
                           t += V[i][k]*V[i][j];
                        }
                        t = -t/V[k+1][k];
                        for (i = k+1; i < n; i++) {
                           V[i][j] += t*V[i][k];
                        }
                     }
                  }
                  for (i = 0; i < n; i++) {
                     V[i][k] = Real(0.0);
                  }
                  V[k][k] = Real(1.0);
               }
            }

            // Main iteration loop for the singular values.

            int pp = p-1;
            int iter = 0;
            double eps = pow(2.0,-52.0);
            while (p > 0) {
               int k=0;
               int kase=0;

               // Here is where a test for too many iterations would go.

               // This section of the program inspects for
               // negligible elements in the s and e arrays.  On
               // completion the variables kase and k are set as follows.

               // kase = 1     if s(p) and e[k-1] are negligible and k<p
               // kase = 2     if s(k) is negligible and k<p
               // kase = 3     if e[k-1] is negligible, k<p, and
               //              s(k), ..., s(p) are not negligible (qr step).
               // kase = 4     if e(p-1) is negligible (convergence).

               for (k = p-2; k >= -1; k--) {
                  if (k == -1) {
                     break;
                  }
                  if (abs(e[k]) <= eps*(abs(s[k]) + abs(s[k+1]))) {
                     e[k] = Real(0.0);
                     break;
                  }
               }
               if (k == p-2) {
                  kase = 4;
               } else {
                  int ks;
                  for (ks = p-1; ks >= k; ks--) {
                     if (ks == k) {
                        break;
                     }
                     double t = (ks != p ? abs(e[ks]) : 0.) + 
                        (ks != k+1 ? abs(e[ks-1]) : 0.);
                     if (abs(s[ks]) <= eps*t)  {
                        s[ks] = Real(0.0);
                        break;
                     }
                  }
                  if (ks == k) {
                     kase = 3;
                  } else if (ks == p-1) {
                     kase = 1;
                  } else {
                     kase = 2;
                     k = ks;
                  }
               }
               k++;

               // Perform the task indicated by kase.

               switch (kase) {

                  // Deflate negligible s(p).

                  case 1: {
                     double f = e[p-2];
                     e[p-2] = Real(0.0);
                     for (j = p-2; j >= k; j--) {
                        double t = hypot(s[j],f);
                        double cs = s[j]/t;
                        double sn = f/t;
                        s[j] = t;
                        if (j != k) {
                           f = -sn*e[j-1];
                           e[j-1] = cs*e[j-1];
                        }
                        if (wantv) {
                           for (i = 0; i < n; i++) {
                              t = cs*V[i][j] + sn*V[i][p-1];
                              V[i][p-1] = -sn*V[i][j] + cs*V[i][p-1];
                              V[i][j] = t;
                           }
                        }
                     }
                  }
                     break;

                     // Split at negligible s(k).

                  case 2: {
                     double f = e[k-1];
                     e[k-1] = Real(0.0);
                     for (j = k; j < p; j++) {
                        double t = hypot(s[j],f);
                        double cs = s[j]/t;
                        double sn = f/t;
                        s[j] = t;
                        f = -sn*e[j];
                        e[j] = cs*e[j];
                        if (wantu) {
                           for (i = 0; i < m; i++) {
                              t = cs*U[i][j] + sn*U[i][k-1];
                              U[i][k-1] = -sn*U[i][j] + cs*U[i][k-1];
                              U[i][j] = t;
                           }
                        }
                     }
                  }
                     break;

                     // Perform one qr step.

                  case 3: {

                     // Calculate the shift.
   
                     double scale = max(max(max(max(
                                                   abs(s[p-1]),abs(s[p-2])),abs(e[p-2])), 
                                            abs(s[k])),abs(e[k]));
                     double sp = s[p-1]/scale;
                     double spm1 = s[p-2]/scale;
                     double epm1 = e[p-2]/scale;
                     double sk = s[k]/scale;
                     double ek = e[k]/scale;
                     double b = ((spm1 + sp)*(spm1 - sp) + epm1*epm1)/2.0;
                     double c = (sp*epm1)*(sp*epm1);
                     double shift = Real(0.0);
                     if ((b != Real(0.0)) || (c != Real(0.0))) {
                        shift = sqrt(b*b + c);
                        if (b < Real(0.0)) {
                           shift = -shift;
                        }
                        shift = c/(b + shift);
                     }
                     double f = (sk + sp)*(sk - sp) + shift;
                     double g = sk*ek;
   
                     // Chase zeros.
   
                     for (j = k; j < p-1; j++) {
                        double t = hypot(f,g);
                        double cs = f/t;
                        double sn = g/t;
                        if (j != k) {
                           e[j-1] = t;
                        }
                        f = cs*s[j] + sn*e[j];
                        e[j] = cs*e[j] - sn*s[j];
                        g = sn*s[j+1];
                        s[j+1] = cs*s[j+1];
                        if (wantv) {
                           for (i = 0; i < n; i++) {
                              t = cs*V[i][j] + sn*V[i][j+1];
                              V[i][j+1] = -sn*V[i][j] + cs*V[i][j+1];
                              V[i][j] = t;
                           }
                        }
                        t = hypot(f,g);
                        cs = f/t;
                        sn = g/t;
                        s[j] = t;
                        f = cs*e[j] + sn*s[j+1];
                        s[j+1] = -sn*e[j] + cs*s[j+1];
                        g = sn*e[j+1];
                        e[j+1] = cs*e[j+1];
                        if (wantu && (j < m-1)) {
                           for (i = 0; i < m; i++) {
                              t = cs*U[i][j] + sn*U[i][j+1];
                              U[i][j+1] = -sn*U[i][j] + cs*U[i][j+1];
                              U[i][j] = t;
                           }
                        }
                     }
                     e[p-2] = f;
                     iter = iter + 1;
                  }
                     break;

                     // Convergence.

                  case 4: {

                     // Make the singular values positive.
   
                     if (s[k] <= Real(0.0)) {
                        s[k] = (s[k] < Real(0.0) ? -s[k] : Real(0.0));
                        if (wantv) {
                           for (i = 0; i <= pp; i++) {
                              V[i][k] = -V[i][k];
                           }
                        }
                     }
   
                     // Order the singular values.
   
                     while (k < pp) {
                        if (s[k] >= s[k+1]) {
                           break;
                        }
                        double t = s[k];
                        s[k] = s[k+1];
                        s[k+1] = t;
                        if (wantv && (k < n-1)) {
                           for (i = 0; i < n; i++) {
                              t = V[i][k+1]; V[i][k+1] = V[i][k]; V[i][k] = t;
                           }
                        }
                        if (wantu && (k < m-1)) {
                           for (i = 0; i < m; i++) {
                              t = U[i][k+1]; U[i][k+1] = U[i][k]; U[i][k] = t;
                           }
                        }
                        k++;
                     }
                     iter = 0;
                     p--;
                  }
                     break;
               }
            }
         }

         Matrix<Real> getU() const
         {
            int minm = min(m+1,n);

            Matrix<Real> res(m, minm);

            for (int i=0; i<m; i++)
               for (int j=0; j<minm; j++)
                  res[i][j] = U[i][j];

            return res;
         }

         Matrix<Real> const& getV() const { return V; }
            
         Matrix<Real> getS() const
         {

            Matrix<Real> res(m, n);
            makeZeroMatrix(res);

            for (int i = 0; i < n; i++)
               res[i][i] = s[i];

            return res;
         }

         void getU (Matrix<Real> &A) 
         {
            int minm = min(m+1,n);

            A = Matrix<Real>(m, minm);

            for (int i=0; i<m; i++)
               for (int j=0; j<minm; j++)
                  A[i][j] = U[i][j];
         }

         /* Return the right singular vectors */

         void getV (Matrix<Real> &A) 
         {
            A = V;
         }

         /** Return the one-dimensional array of singular values */

         void getSingularValues (Vector<Real> &x) 
         {
            x = s;
         }

         Vector<Real> const & getSingularValues() const
         {
            return this->s;
         }

         /** Return the diagonal matrix of singular values
             @return     S
         */

         void getS (Matrix<Real> &A) {
            A = Matrix<Real>(n,n);
            for (int i = 0; i < n; i++) {
               for (int j = 0; j < n; j++) {
                  A[i][j] = Real(0.0);
               }
               A[i][i] = s[i];
            }
         }

         /** Two norm  (max(S)) */

         double norm2 () {
            return s[0];
         }

         /** Two norm of condition number (max(S)/min(S)) */

         double cond () {
            return s[0]/s[min(m,n)-1];
         }

         /** Effective numerical matrix rank
             @return     Number of nonnegligible singular values.
         */

         int rank () 
         {
            double eps = pow(2.0,-52.0);
            double tol = max(m,n)*s[0]*eps;
            int r = 0;
            for (int i = 0; i < s.dim(); i++) {
               if (s[i] > tol) {
                  r++;
               }
            }
            return r;
         }
   }; // end struct SVD

} // end namespace V3D

#endif
