// -*- C++ -*-
#ifndef V3D_STEREOBUNDLE_H
#define V3D_STEREOBUNDLE_H

# if defined(V3DLIB_ENABLE_SUITESPARSE)

#include "Math/v3d_optimization.h"
#include "Math/v3d_linear.h"
#include "Geometry/v3d_cameramatrix.h"

namespace V3D
{

   /*! Although it is called StereoMetricBundleOptimizer, it really optimized general
    * rigidly moving multi-camera systems (binocular, trinocular etc.).
    * View refer here to poses of the rigid system; subcamera indices denote the particular
    * camera on the platform (e.g. 0 and 1 for binocular rigs).
    * The poses of the moving platforms and the 3d points are optimized.
    * If X_j is visible in view i with pose RT_i in subcamera l (with projection P_l),
    * then the projected measurements is p = P_l RT_i X_j.
    */
   struct StereoMetricBundleOptimizer : public SparseLevenbergOptimizer
   {
         typedef SparseLevenbergOptimizer Base;

         StereoMetricBundleOptimizer(double inlierThreshold,
                                     vector<Matrix3x3d>& rotations, // The poses of the rigs (rotation part),
                                     vector<Vector3d>& translations, // translation part
                                     vector<CameraMatrix> const& rigCameras, // poses of the cameras on the rig
                                     vector<Vector3d>& Xs,
                                     vector<Vector2d> const& measurements,
                                     vector<int> const& corrspondingView,
                                     vector<int> const& corrspondingPoint,
                                     vector<int> const& correspondingSubCamera)
            : SparseLevenbergOptimizer(2, rotations.size(), 6, Xs.size(), 3, 0,
                                       corrspondingView, corrspondingPoint),
              _rotations(rotations), _translations(translations),
              _Xs(Xs), _measurements(measurements),
              _rigCameras(rigCameras), _correspondingSubCamera(correspondingSubCamera),
              _savedTranslations(rotations.size()), _savedRotations(rotations.size()),
              _savedXs(Xs.size()),
              _inlierThreshold(inlierThreshold), _cachedParamLength(0.0)
         {
            // Since we assume that BA does not alter the inputs too much,
            // we compute the overall length of the parameter vector in advance
            // and return that value as the result of getParameterLength().
            for (int i = _nNonvaryingA; i < _nParametersA; ++i)
            {
               _cachedParamLength += sqrNorm_L2(_translations[i]);
               _cachedParamLength += 3.0; // Assume eye(3) for R.
            }
            for (int j = _nNonvaryingB; j < _nParametersB; ++j)
               _cachedParamLength += sqrNorm_L2(_Xs[j]);

            _cachedParamLength = sqrt(_cachedParamLength);
         }

         // Huber robust cost function.
         virtual void fillWeights(VectorArray<double> const& residual, Vector<double>& w)
         {
            for (unsigned int k = 0; k < w.size(); ++k)
            {
               Vector<double> const& r = residual[k];
               double const e = norm_L2(r);
               w[k] = (e < _inlierThreshold) ? 1.0 : sqrt(_inlierThreshold / e);
            } // end for (k)
         }

         virtual void evalResidual(VectorArray<double>& e)
         {
            Vector3d X;

            for (unsigned int k = 0; k < e.count(); ++k)
            {
               int const i = _correspondingParamA[k];
               int const j = _correspondingParamB[k];
               int const l = _correspondingSubCamera[k];

               multiply_A_v(_rotations[i], _Xs[j], X); // Transform into rig space
               X = X + _translations[i];
               Vector2d const q = _rigCameras[l].projectPoint(X); // project into respective camera of the rig
               e[k][0] = q[0] - _measurements[k][0];
               e[k][1] = q[1] - _measurements[k][1];
            }
         }

         virtual void fillJacobians(Matrix<double>& Ak, Matrix<double>& Bk, Matrix<double>& Ck,
                                    int i, int j, int k);

         virtual double getParameterLength() const
         {
            return _cachedParamLength;
         }

         void updateParametersA(VectorArray<double> const& deltaAi)
         {
            Vector3d T, omega;
            Matrix3x3d R0, dR;

            for (int i = _nNonvaryingA; i < _nParametersA; ++i)
            {
               _translations[i][0] += deltaAi[i][0];
               _translations[i][1] += deltaAi[i][1];
               _translations[i][2] += deltaAi[i][2];

               // Create incremental rotation using Rodriguez formula.
               R0 = _rotations[i];
               omega[0] = deltaAi[i][3];
               omega[1] = deltaAi[i][4];
               omega[2] = deltaAi[i][5];
               createRotationMatrixRodrigues(omega, dR);
               _rotations[i] = dR * R0;
            } // end for (i)
         } // end updateParametersA()

         void updateParametersB(VectorArray<double> const& deltaBj)
         {
            for (int j = _nNonvaryingB; j < _nParametersB; ++j)
            {
               _Xs[j][0] += deltaBj[j][0];
               _Xs[j][1] += deltaBj[j][1];
               _Xs[j][2] += deltaBj[j][2];
            }
         } // end updateParametersB()

         virtual void updateParametersC(Vector<double> const& deltaC) { }

         virtual void saveAllParameters()
         {
            for (int i = _nNonvaryingA; i < _nParametersA; ++i)
            {
               _savedTranslations[i] = _translations[i];
               _savedRotations[i]    = _rotations[i];
            }
            _savedXs = _Xs;
         }

         virtual void restoreAllParameters()
         {
            for (int i = _nNonvaryingA; i < _nParametersA; ++i)
            {
               _translations[i] = _savedTranslations[i];
               _rotations[i]    = _savedRotations[i];
            }
            _Xs = _savedXs;
         }

      protected:
         vector<Matrix3x3d>& _rotations;
         vector<Vector3d>&   _translations;
         vector<Vector3d>&   _Xs;

         vector<Vector2d>     const& _measurements; 
         vector<CameraMatrix> const& _rigCameras;
         vector<int>          const& _correspondingSubCamera;

         vector<Vector3d>   _savedTranslations;
         vector<Matrix3x3d> _savedRotations;
         vector<Vector3d>   _savedXs;

         double const _inlierThreshold;
         double       _cachedParamLength;
   }; // end struct StereoMetricBundleOptimizer

} // end namespace V3D

# endif

#endif
