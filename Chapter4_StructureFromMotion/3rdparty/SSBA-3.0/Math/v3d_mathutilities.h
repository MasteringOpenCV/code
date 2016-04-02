// -*- C++ -*-

#ifndef V3D_MATH_UTILITIES_H
#define V3D_MATH_UTILITIES_H

#include "Math/v3d_linear.h"
#include "Math/v3d_linear_tnt.h"

#include <vector>

#ifdef WIN32
#include "win32config.h"
#endif

namespace V3D
{

   template <typename Mat>
   inline double
   getRotationMagnitude(Mat const& R)
   {
      assert(R.num_rows() == 3);
      assert(R.num_cols() == 3);

      double cos_theta = 0.5 * (R[0][0] + R[1][1] + R[2][2] - 1.0);
      cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
      return acos(cos_theta);
   }

   template <typename Mat>
   inline Vector3d
   getRotationAxis(Mat const& R)
   {
      assert(R.num_rows() == 3);
      assert(R.num_cols() == 3);

      Vector3d axis(0.0, 0.0, 1.0);

      double const two_sin_theta = sqrt(sqr(R[1][0] - R[0][1]) + sqr(R[0][2] - R[2][0]) + sqr(R[2][1] - R[1][2]));

      if (fabs(two_sin_theta) > 1e-10)
      {
#if 0
         axis[0] = (R[1][0] - R[0][1]) / two_sin_theta;
         axis[1] = (R[0][2] - R[2][0]) / two_sin_theta;
         axis[2] = (R[2][1] - R[1][2]) / two_sin_theta;
#else
         // Patch by Bastien
         axis[0] = (R[2][1] - R[1][2]) / two_sin_theta;
         axis[1] = (R[0][2] - R[2][0]) / two_sin_theta;
         axis[2] = (R[1][0] - R[0][1]) / two_sin_theta;
#endif
      }
      return axis;
   }

#if 0
   template <typename Mat, typename Vec4>
   inline void
   createQuaternionFromRotationMatrix(Mat const& R_, Vec4& q)
   {
      assert(R_.num_rows() == 3);
      assert(R_.num_cols() == 3);
      assert(q.size() == 4);

      Mat R;
      makeTransposedMatrix(R_, R);

      const double tr = R[0][0] + R[1][1] + R[2][2];

      if (tr >= 0.0)
      {
         double s = sqrt(tr + 1);
         q[3] = s * 0.5;
         s = 0.5 / s;

         q[0] = (R[1][2] - R[2][1]) * s;
         q[1] = (R[2][0] - R[0][2]) * s;
         q[2] = (R[0][1] - R[1][0]) * s;
      }
      else
      {
         int i = 0;
         if (R[1][1] > R[0][0]) i = 1;
         if (R[2][2] > R[i][i]) i = 2;

         if (tr > R[i][i])
         {
            // Compute w first:
            q[3] = sqrt(tr + 1)/2.0;
            double const w4 = 4 * q[3];

            // And compute other values:
            q[0] = (R[1][2] - R[2][1]) / w4;
            q[1] = (R[2][0] - R[0][2]) / w4;
            q[2] = (R[0][1] - R[1][0]) / w4;
         }
         else
         {
            // Compute x, y, or z first:
            int j = (i+1) % 3;
            int k = (j+1) % 3;

            // Compute first value:
            q[i] = sqrt(R[i][i] - R[j][j] - R[k][k] + 1) / 2;

            // And the others:
            q[j] = (R[j][i] + R[i][j]) / (4*q[i]);
            q[k] = (R[k][i] + R[i][k]) / (4*q[i]);

            q[3] = (R[j][k]-R[k][j])/(4*q[i]);
         } // end if
      } // end if (tr > 0)
   } // end createQuaternionFromRotationMatrix()
#else
   inline double _copysign(double x, double y)
   {
      if (y < 0)
         return -fabs(x);
      else
         return fabs(x);
   }

   template <typename Mat, typename Vec4>
   inline void
   createQuaternionFromRotationMatrix(Mat const& R, Vec4& q)
   {
      assert(R.num_rows() == 3);
      assert(R.num_cols() == 3);
      assert(q.size() == 4);

      double const m00 = R[0][0]; double const m01 = R[0][1]; double const m02 = R[0][2];
      double const m10 = R[1][0]; double const m11 = R[1][1]; double const m12 = R[1][2];
      double const m20 = R[2][0]; double const m21 = R[2][1]; double const m22 = R[2][2];

      q[3] = sqrt(std::max(0.0, 1.0 + m00 + m11 + m22)) / 2;
      q[0] = sqrt(std::max(0.0, 1.0 + m00 - m11 - m22)) / 2;
      q[1] = sqrt(std::max(0.0, 1.0 - m00 + m11 - m22)) / 2;
      q[2] = sqrt(std::max(0.0, 1.0 - m00 - m11 + m22)) / 2;

      q[0] = _copysign(q[0], m21 - m12);
      q[1] = _copysign(q[1], m02 - m20);
      q[2] = _copysign(q[2], m10 - m01);
   } // end createQuaternionFromRotationMatrix()
#endif

   template <typename Mat, typename Vec4>
   inline void
   createRotationMatrixFromQuaternion(Vec4 const& q, Mat& R)
   {
      assert(R.num_rows() == 3);
      assert(R.num_cols() == 3);
      assert(q.size() == 4);

      double x = q[0];
      double y = q[1];
      double z = q[2];
      double w = q[3];

      double const len = sqrt(x*x + y*y + z*z + w*w);
      double const s = (len > 0.0) ? (1.0 / len) : 0.0;

      x *= s; y *= s; z *= s; w *= s;

      double const wx = 2*w*x; double const wy = 2*w*y; double const wz = 2*w*z;
      double const xx = 2*x*x; double const xy = 2*x*y; double const xz = 2*x*z;
      double const yy = 2*y*y; double const yz = 2*y*z; double const zz = 2*z*z;
      R[0][0] = 1.0 - (yy+zz); R[0][1] = xy-wz;         R[0][2] = xz+wy;
      R[1][0] = xy+wz;         R[1][1] = 1.0 - (xx+zz); R[1][2] = yz-wx;
      R[2][0] = xz-wy;         R[2][1] = yz+wx;         R[2][2] = 1.0 - (xx+yy);
   } // end createRotationMatrixFromQuaternion()

   template <typename Vec, typename Mat>
   inline void
   createRotationMatrixRodrigues(Vec const& omega, Mat& R)
   {
      assert(omega.size() == 3);
      assert(R.num_rows() == 3);
      assert(R.num_cols() == 3);

      double const theta = norm_L2(omega);
      makeIdentityMatrix(R);
      if (fabs(theta) > 1e-6)
      {
         Matrix3x3d J, J2;
         makeCrossProductMatrix(omega, J);
         multiply_A_B(J, J, J2);
         double const c1 = sin(theta)/theta;
         double const c2 = (1-cos(theta))/(theta*theta);
         for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
               R[i][j] += c1*J[i][j] + c2*J2[i][j];
      }
   } // end createRotationMatrixRodrigues()

   template <typename Vec, typename Mat>
   inline void
   createRodriguesParamFromRotationMatrix(Mat const& R, Vec& omega)
   {
      assert(omega.size() == 3);
      assert(R.num_rows() == 3);
      assert(R.num_cols() == 3);

      Vector4d q;
      createQuaternionFromRotationMatrix(R, q);
      omega[0] = q[0];
      omega[1] = q[1];
      omega[2] = q[2];
      normalizeVector(omega);
      scaleVectorIP(2.0*acos(q[3]), omega);
   } // end createRodriguesParamFromRotationMatrix()
   

   template <typename Vec, typename Mat>
   inline void
   createRotationMatrixFromModifiedRodrigues(Vec const& sigma, Mat& R)
   {
      assert(sigma.size() == 3);
      assert(R.num_rows() == 3);
      assert(R.num_cols() == 3);

      double const s2 = sqrNorm_L2(sigma);

      makeIdentityMatrix(R);
      Matrix3x3d J, J2;
      makeCrossProductMatrix(sigma, J);
      multiply_A_B(J, J, J2);
      Matrix3x3d tmp = 8.0*J2 - 4.0*(1.0 - s2)*J;
      scaleMatrixIP(1/(1+s2)/(1+s2), tmp);
      addMatrices(R, tmp, R);
   } // end createRotationMatrixFromModifiedRodrigues()

   template <typename Vec, typename Mat>
   inline void
   createModifiedRodriguesParamFromRotationMatrix(Mat const& R, Vec& sigma)
   {
      assert(sigma.size() == 3);
      assert(R.num_rows() == 3);
      assert(R.num_cols() == 3);

      Vector3d omega;
      createRodriguesParamFromRotationMatrix(R, omega);
      double w2 = sqrNorm_L2(omega);
      double denom = 1.0 / (1 + sqrt(1.0 + w2));
      sigma[0] = omega[0] * denom;
      sigma[1] = omega[1] * denom;
      sigma[2] = omega[2] * denom;
   } // end createModifiedRodriguesParamFromRotationMatrix()

   template <typename Vec3, typename Vec4>
   void
   createQuaternionForVectorAlignment(Vec3 const& src_, Vec3 const& dst_, Vec4& q)
   {
      assert(src_.size() == 3);
      assert(dst_.size() == 3);
      assert(q.size() == 4);

      Vector3d src, dst, axis;
      copyVector(src_, src);
      copyVector(dst_, dst);

      normalizeVector(src);
      normalizeVector(dst);

      double dot = innerProduct(src, dst);
      makeCrossProductVector(src, dst, axis);
      const double axislen2 = sqrNorm_L2(axis);

      if (axislen2 == 0.0)
      {
         // src and dst are parallel
         // Check if they are pointing in the same direction.
         if (dot > 0.0)
         {
            q[0] = q[1] = q[2] = 0.0; q[3] = 1.0;
         }
         // Ok, so they are parallel and pointing in the opposite direction
         // of each other.
         else
         {
            Vector3d t;
            // Try crossing with x axis.
            makeCrossProductVector(src, makeVector3(1.0, 0.0, 0.0), t);
            // If not ok, cross with y axis.
            if(sqrNorm_L2(t) == 0.0) makeCrossProductVector(src, makeVector3(0.0, 1.0, 0.0), t);

            normalizeVector(t);
            q[0] = t[0]; q[1] = t[1]; q[2] = t[2]; q[3] = 0.0;
         } // end if
      }
      else
      {
         normalizeVector(axis);
         dot = std::max(-1.0, std::min(1.0, dot));
         // use half-angle formulae
         // sin^2 t = ( 1 - cos (2t) ) / 2
         scaleVectorIP(sqrt((1.0 - dot) / 2), axis);

         q[0] = axis[0];
         q[1] = axis[1];
         q[2] = axis[2];

         // cos^2 t = ( 1 + cos (2t) ) / 2
         // w part is cosine of half the rotation angle
         q[3] = sqrt((1.0 + dot) / 2);
      } // end if
   } // end createQuaternionForVectorAlignment()

   inline void
   enforceRotationMatrix(Matrix<double>& R)
   {
      assert(R.num_rows() == 3);
      assert(R.num_cols() == 3);

      SVD<double> svd(R);
      multiply_A_Bt(svd.getU(), svd.getV(), R);
   }

   template <typename Mat>
   inline void
   enforceRotationMatrix(Mat& R)
   {
      assert(R.num_rows() == 3);
      assert(R.num_cols() == 3);

      Matrix<double> RR(3, 3);
      copyMatrix(R, RR);
      enforceRotationMatrix(RR);
      copyMatrix(RR, R);
   }

   inline void
   getEuclideanTransformation(vector<Vector3d> const& left_, vector<Vector3d> const& right_,
                              Matrix3x3d& R, Vector3d& T)
   {
      assert(left_.size() == right_.size());

      size_t const n = left_.size();

      // Determine centroids and shifted points
      vector<Vector3d> left(n);
      vector<Vector3d> right(n);

      Vector3d leftMean, rightMean;
      makeZeroVector(leftMean);
      makeZeroVector(rightMean);

      for (size_t i = 0; i < n; ++i)
      {
         leftMean  = leftMean + left_[i];
         rightMean = rightMean + right_[i];
      }
      scaleVectorIP(1.0 / n, leftMean);
      scaleVectorIP(1.0 / n, rightMean);

      for (size_t i = 0; i < n; ++i)
      {
         left[i]  = left_[i] - leftMean;
         right[i] = right_[i] - rightMean;
      }

      Matrix3x3d H, outer;
      makeZeroMatrix(H);

      for (size_t i = 0; i < n; ++i)
      {
         makeOuterProductMatrix(right[i], left[i], outer);
         addMatricesIP(outer, H);
      }

      Matrix<double> HH(3, 3);
      copyMatrix(H, HH);
      SVD<double> svd(HH);

      Matrix<double> const  U = svd.getU();
      Matrix<double> const& V = svd.getV();

      multiply_A_Bt(U, V, R);
      if (matrixDeterminant3x3(R) < 0)
      {
         Matrix3x3d V1;
         copyMatrix(V, V1);
         V1[0][2] = -V1[0][2];
         V1[1][2] = -V1[1][2];
         V1[2][2] = -V1[2][2];
         multiply_A_Bt(U, V1, R);
      }
      T = rightMean - R*(leftMean);
   } // end getEuclideanTransformation()

   // The transformation from left to right is given by Y = scale * (R*X + T).
   inline void
   getSimilarityTransformation(vector<Vector3d> const& left_, vector<Vector3d> const& right_,
                               Matrix3x3d& R, Vector3d& T, double& scale)
   {
      assert(left_.size() == right_.size());

      size_t const n = left_.size();

      vector<Vector3d> left(left_);
      vector<Vector3d> right(right_);

      Vector3d leftMean, rightMean;
      makeZeroVector(leftMean);
      makeZeroVector(rightMean);

      for (size_t i = 0; i < n; ++i)
      {
         leftMean  = leftMean + left_[i];
         rightMean = rightMean + right_[i];
      }
      scaleVectorIP(1.0 / n, leftMean);
      scaleVectorIP(1.0 / n, rightMean);

      scale = 0.0;
      for (size_t i = 0; i < n; ++i)
      {
         double d1 = distance_L2(left[i], leftMean);
         double d2 = distance_L2(right[i], rightMean);
         scale += d2/d1;
      }
      scale /= n;
      if(isinf(scale)||isnan(scale)||scale==0.0)scale=1.0;
      for (size_t i = 0; i < n; ++i) scaleVectorIP(scale, left[i]);

      getEuclideanTransformation(left, right, R, T);
      scaleVectorIP(1.0 / scale, T);
   } // end getSimilarityTransformation()

   inline void
   makePseudoInverse(Matrix<double> const& A, Matrix<double>& Aplus, double tolerance = 1e-12)
   {
      int const M = A.num_rows();
      int const N = A.num_cols();

      SVD<double> svd(A);

      Matrix<double> U = svd.getU();
      Matrix<double> V = svd.getV();
      Vector<double> const& S = svd.getSingularValues();

      double eps = tolerance * S[0];
      for (int i = 0; i < N; ++i)
      {
         double const splus = (S[i] < eps) ? 0.0 : 1.0 / S[i];

         for (int j = 0; j < N; ++j) V[j][i] *= splus;
      }

      Aplus.newsize(N, M);
      multiply_A_Bt(V, U, Aplus);
   } // end makePseudoInverse()

   template <typename Field>
   struct RootFindingParameters
   {
         RootFindingParameters()
            : maxBracketingExponent(32), maxBisectionIterations(800),
              coefficientTolerance(1e-12), rootTolerance(1e-20)
         { }

         int maxBracketingExponent; //!< max power of 10 we wish to search to
         int maxBisectionIterations; //!< max number of iterations in the bisection routine
         Field coefficientTolerance; //!< a coefficient smaller than this is considered to be exactly zero
         Field rootTolerance; //!< smallest relative error we want
   }; // end struct RootFindingParameters

   template <typename Field>
   Field evalPolynomial(int order, Field const * coeffs, Field z);

   int getRealRootsOfQuadraticPolynomial(double a, double b, double c, double roots[2]);

   int getRealRootsOfCubicPolynomial(double a, double b, double c, double d, double roots[3]);

   int getRealRootsOfQuarticPolynomial(double a, double b, double c, double d, double e, double roots[4]);

   //! \brief Compute all real roots of the given polynomial first by
   //! bracketing them and then by applying the regula-falsi method.
   /*! For some degenerate cases the root bracketing/counting will fail, the returned result
    * is false is those cases.
    */
   template <typename Field>
   bool computeRealRootsOfPolynomial(int order, Field const * coeffs, std::vector<Field>& roots,
                                     RootFindingParameters<Field> const& params = RootFindingParameters<Field>());

   template <typename T> inline T sqr(T x) { return x*x; }

//**********************************************************************

   struct SimilarityTransform
   {
         double     scale;
         Matrix3x3d R;
         Vector3d   T;

         SimilarityTransform()
            : scale(1.0)
         {
            makeIdentityMatrix(R);
            makeZeroVector(T);
         }

         SimilarityTransform(double scale_, Matrix3x3d const& R_, Vector3d const& T_)
            : scale(scale_)
         {
            copyMatrix(R_, R);
            copyVector(T_, T);
         }

         operator Matrix4x4d() const
         {
            Matrix4x4d res;
            copyMatrixSlice(R, 0, 0, 3, 3, res, 0, 0);
            res[0][3] = T[0]; res[1][3] = T[1]; res[2][3] = T[2];
            scaleMatrixIP(scale, res);
            res[3][0] = res[3][1] = res[3][2] = 0.0; res[3][3] = 1;
            return res;
         }

         Vector3d apply(Vector3d const& X) const
         {
            return scale * (R*X + T);
         }

         SimilarityTransform inverse() const
         {
            return SimilarityTransform(1.0/scale, R.transposed(), -scale*(R.transposed()*T));
         }
         template <typename Archive> void serialize(Archive& ar)
         {
            V3D::SerializationScope<Archive> scope(ar, "SimilarityTransform");
            ar &  scale & T & R;
         }
         V3D_DEFINE_LOAD_SAVE(SimilarityTransform)
   }; // end struct SimilarityTransform

   // Composition of transforms
   inline SimilarityTransform operator*(SimilarityTransform const& T2, SimilarityTransform const& T1)
   {
      return SimilarityTransform(T2.scale*T1.scale, T2.R * T1.R, T2.R*T1.T + (1.0/T1.scale)*T2.T);
   }

   inline void
   displayTransform(SimilarityTransform const& xform)
   {
      using namespace std;

      cout << "scale = " << xform.scale << endl;
      cout << "R = "; displayMatrix(xform.R);
      cout << "T = "; displayVector(xform.T);
   }

} // namespace V3D

#endif
