#include "Geometry/v3d_stereobundle.h"

#if defined(V3DLIB_ENABLE_SUITESPARSE)

using namespace V3D;

namespace
{
   typedef InlineMatrix<double, 3, 6> Matrix3x6d;

   void
   poseDerivatives(Matrix3x3d const& R, Vector3d const& T, CameraMatrix const& rigCam, Vector3d const& X,
                   Vector3d& XX, Matrix3x6d& d_dRT, Matrix3x3d& d_dX)
   {
      Vector3d const RX = R * X;
      Vector3d const Y = RX + T;
      Matrix3x6d dY_dRT;

      // See Frank Dellaerts bundle adjustment tutorial.
      // d(dR * R0 * X + t)/d omega = -[R0 * X]_x
      Matrix3x3d J;
      makeCrossProductMatrix(RX, J);
      scaleMatrixIP(-1.0, J);

      // Now the transformation from world coords into camera space is xx = Rx + T
      // Hence the derivative of x wrt. T is just the identity matrix.
      makeIdentityMatrix(dY_dRT);
      copyMatrixSlice(J, 0, 0, 3, 3, dY_dRT, 0, 3);

      XX = rigCam.transformPointIntoCameraSpace(Y);

      // dXX/dY is just rigCam.getRotation(); dXX/dRT = dXX/dY * dY/dRT
      multiply_A_B(rigCam.getRotation(), dY_dRT, d_dRT);

      // The derivative of Rx+T wrt x is just R -- here R is the combined rotation.
      multiply_A_B(rigCam.getRotation(), R, d_dX);
   } // end poseDerivatives()

} // end namespace <>

namespace V3D
{

   void
   StereoMetricBundleOptimizer::fillJacobians(Matrix<double>& Ak, Matrix<double>& Bk, Matrix<double>& Ck,
                                              int i, int j, int k)
   {
      int const l = _correspondingSubCamera[k];

      Vector3d XX;
      Matrix3x6d d_dRT;
      Matrix3x3d d_dX;

      ::poseDerivatives(_rotations[i], _translations[i], _rigCameras[l], _Xs[j], XX, d_dRT, d_dX);

      double const f  = _rigCameras[l].getFocalLength();
      double const ar = _rigCameras[l].getAspectRatio();

      Matrix2x3d dp_dX;
      double const bx = f / (XX[2] * XX[2]);
      double const by = ar * bx;
      dp_dX[0][0] = bx * XX[2]; dp_dX[0][1] = 0;          dp_dX[0][2] = -bx * XX[0];
      dp_dX[1][0] = 0;          dp_dX[1][1] = by * XX[2]; dp_dX[1][2] = -by * XX[1];

      multiply_A_B(dp_dX, d_dRT, Ak);
      multiply_A_B(dp_dX, d_dX, Bk);
   } // end StereoMetricBundleOptimizer::fillJacobians()

} // end namespace V3D

#endif
