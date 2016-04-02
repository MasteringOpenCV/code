// Bundle adjustment application for datasets captured with different cameras (varying intrinsics).

#include "Base/v3d_vrmlio.h"
#include "Math/v3d_linear.h"
#include "Math/v3d_nonlinlsq.h"
#include "Geometry/v3d_mviewutilities.h"
#include "Geometry/v3d_distortion.h"
#include "Geometry/v3d_metricbundle.h"

#include <iostream>
#include <fstream>
#include <map>

using namespace V3D;
using namespace std;

namespace
{

#define CAMERA_PARAM_TYPE 0
#define POINT_PARAM_TYPE 1

   typedef V3D::InlineMatrix<double, 2, 6> Matrix2x6d;
   typedef V3D::InlineMatrix<double, 3, 6> Matrix3x6d;

   struct VaryingIntrinsicBundleCostFunction : public NLSQ_CostFunction
   {
         static int extParamDimensionFromMode(int mode)
         {
            switch (mode)
            {
               case FULL_BUNDLE_NO_ROTATIONS:
               case FULL_BUNDLE_METRIC:            return 0;
               case FULL_BUNDLE_FOCAL_LENGTH:      return 1;
               case FULL_BUNDLE_FOCAL_LENGTH_PP:   return 3;
               case FULL_BUNDLE_RADIAL:            return 5;
               case FULL_BUNDLE_RADIAL_TANGENTIAL: return 7;
            }
            return 0;
         }

         VaryingIntrinsicBundleCostFunction(int const mode,
                                            std::vector<int> const& usedParamTypes,
                                            double inlierThreshold,
                                            std::vector<StdDistortionFunction> const& distortions,
                                            vector<CameraMatrix> const& cams,
                                            vector<Vector3d >& Xs,
                                            vector<Vector2d > const& measurements,
                                            Matrix<int> const& correspondingParams)
            : NLSQ_CostFunction(usedParamTypes, correspondingParams, 2),
              _cams(cams), _distortions(distortions), _Xs(Xs), _measurements(measurements),
              _inlierThreshold(inlierThreshold), _mode(mode)
         {
            assert(usedParamTypes.size() == 2);
         }

         Vector2d projectPoint(Vector3d const& X, int i) const
         {
            return _cams[i].projectPoint(_distortions[i], X);
         }

         virtual void evalResidual(int const k, Vector<double>& e) const
         {
            unsigned view = _correspondingParams[k][0];
            unsigned point = _correspondingParams[k][1];

            Vector3d const& X = _Xs[point];

            Vector2d const q = this->projectPoint(_Xs[point], view);
            e[0] = q[0] - _measurements[k][0];
            e[1] = q[1] - _measurements[k][1];
         }

         virtual double getWeight(Vector<double> const& r) const
         {
            double const e = norm_L2(r);
            return (e < _inlierThreshold) ? 1.0 : sqrt(_inlierThreshold / e);
         }

         virtual void fillJacobian(int const whichParam, int const paramIx, int const k, Matrix<double>& Jdst) const
         {
            int const view  = _correspondingParams[k][0];
            int const point = _correspondingParams[k][1];

            double const focalLength = _cams[view].getFocalLength();
            double const aspectRatio = _cams[view].getAspectRatio();

            Vector3d const XX = _cams[view].transformPointIntoCameraSpace(_Xs[point]);
            Vector2d xu; // undistorted image point
            xu[0] = XX[0] / XX[2];
            xu[1] = XX[1] / XX[2];

            Vector2d const xd = _distortions[view](xu); // distorted image point

            Matrix2x2d dp_dxd;
            dp_dxd[0][0] = focalLength; dp_dxd[0][1] = 0;
            dp_dxd[1][0] = 0;           dp_dxd[1][1] = aspectRatio * focalLength;

            // First, lets do the derivative wrt the structure and motion parameters.
            Matrix2x3d dxu_dXX;
            dxu_dXX[0][0] = 1.0f / XX[2]; dxu_dXX[0][1] = 0;            dxu_dXX[0][2] = -XX[0]/(XX[2]*XX[2]);
            dxu_dXX[1][0] = 0;            dxu_dXX[1][1] = 1.0f / XX[2]; dxu_dXX[1][2] = -XX[1]/(XX[2]*XX[2]);

            Matrix2x2d dxd_dxu = _distortions[view].derivativeWrtUndistortedPoint(xu);

            Matrix2x2d dp_dxu = dp_dxd * dxd_dxu;
            Matrix2x3d dp_dXX = dp_dxu * dxu_dXX;

            if (whichParam == 1)
            {
               Matrix3x3d dXX_dX;
               // The derivative of Rx+T wrt x is just R.
               copyMatrix(_cams[view].getRotation(), dXX_dX);

               // Derivative w.r.t. 3D point
               multiply_A_B(dp_dXX, dXX_dX, Jdst);
               return;
            }

            Matrix3x6d dXX_dRT;
            // See Frank Dellaerts bundle adjustment tutorial.
            // d(dR * R0 * X + t)/d omega = -[R0 * X]_x
            Matrix3x3d J;
            makeCrossProductMatrix(XX - _cams[view].getTranslation(), J);
            scaleMatrixIP(-1.0, J);

            // Now the transformation from world coords into camera space is xx = Rx + T
            // Hence the derivative of x wrt. T is just the identity matrix.
            makeIdentityMatrix(dXX_dRT);
            copyMatrixSlice(J, 0, 0, 3, 3, dXX_dRT, 0, 3);

            if (_mode == FULL_BUNDLE_NO_ROTATIONS)
            {
               dXX_dRT[0][0] = dXX_dRT[0][1] = dXX_dRT[0][2] = 0.0;
               dXX_dRT[1][0] = dXX_dRT[1][1] = dXX_dRT[1][2] = 0.0;
               dXX_dRT[2][0] = dXX_dRT[2][1] = dXX_dRT[2][2] = 0.0;
            }

            Matrix2x6d dp_dRT;

            multiply_A_B(dp_dXX, dXX_dRT, dp_dRT);
            copyMatrixSlice(dp_dRT, 0, 0, 2, 6, Jdst, 0, 0);

            switch (_mode)
            {
               case FULL_BUNDLE_RADIAL_TANGENTIAL:
               {
                  Matrix2x2d dxd_dp1p2 = _distortions[view].derivativeWrtTangentialParameters(xu);
                  Matrix2x2d d_dp1p2 = dp_dxd * dxd_dp1p2;
                  copyMatrixSlice(d_dp1p2, 0, 0, 2, 2, Jdst, 0, 11);
                  // No break here!
               }
               case FULL_BUNDLE_RADIAL:
               {
                  Matrix2x2d dxd_dk1k2 = _distortions[view].derivativeWrtRadialParameters(xu);
                  Matrix2x2d d_dk1k2 = dp_dxd * dxd_dk1k2;
                  copyMatrixSlice(d_dk1k2, 0, 0, 2, 2, Jdst, 0, 9);
                  // No break here!
               }
               case FULL_BUNDLE_FOCAL_LENGTH_PP:
               {
                  Jdst[0][7] = 1; Jdst[0][8] = 0;
                  Jdst[1][7] = 0; Jdst[1][8] = 1;
                  // No break here!
               }
               case FULL_BUNDLE_FOCAL_LENGTH:
               {
                  Jdst[0][6] = xd[0];
                  Jdst[1][6] = xd[1];
               }
               case FULL_BUNDLE_METRIC:
               {
               }
            } // end switch
         } // end fillJacobian()

      protected:
         vector<CameraMatrix>          const& _cams;
         vector<StdDistortionFunction> const& _distortions;
         vector<Vector3d >             const& _Xs;

         double                   _inlierThreshold;
         vector<Vector2d > const& _measurements;
         int const                _mode;
   }; // end struct VaryingIntrinsicBundleCostFunction

   struct VaryingIntrinsicBundleOptimizer : public NLSQ_LM_Optimizer
   {
         typedef NLSQ_LM_Optimizer Base;

         VaryingIntrinsicBundleOptimizer(int const mode, NLSQ_ParamDesc const& paramDesc,
                                         std::vector<NLSQ_CostFunction *> const& costFunctions,
                                         std::vector<StdDistortionFunction>& distortions,
                                         vector<CameraMatrix>& cams,
                                         vector<Vector3d >& Xs)
            : Base(paramDesc, costFunctions),
              _mode(mode), _cams(cams), _Xs(Xs), _distortions(distortions),
              _savedTranslations(cams.size()), _savedRotations(cams.size()), _savedKs(cams.size()), _savedDistortions(cams.size()),
              _savedXs(Xs.size()), _cachedParamLength(0.0)
         {
            // Since we assume that BA does not alter the inputs too much,
            // we compute the overall length of the parameter vector in advance
            // and return that value as the result of getParameterLength().
            for (int i = 0; i < _cams.size(); ++i)
            {
               _cachedParamLength += sqrNorm_L2(_cams[i].getTranslation());
               _cachedParamLength += 3.0; // Assume eye(3) for R.
            }
            for (int j = 0; j < _Xs.size(); ++j)
               _cachedParamLength += sqrNorm_L2(_Xs[j]);

            _cachedParamLength = sqrt(_cachedParamLength);
         }

         virtual double getParameterLength() const
         {
            return _cachedParamLength;
         }

         virtual void updateParameters(int const paramType, VectorArrayAdapter<double> const& delta)
         {
            switch (paramType)
            {
               case CAMERA_PARAM_TYPE:
               {
                  Vector3d T, omega;
                  Matrix3x3d R0, dR, K;

                  for (int i = 0; i < _cams.size(); ++i)
                  {
                     T = _cams[i].getTranslation();
                     T[0] += delta[i][0]; T[1] += delta[i][1]; T[2] += delta[i][2];
                     _cams[i].setTranslation(T);

                     // Create incremental rotation using Rodriguez formula.
                     R0 = _cams[i].getRotation();
                     omega[0] = delta[i][3]; omega[1] = delta[i][4]; omega[2] = delta[i][5];
                     createRotationMatrixRodrigues(omega, dR);
                     _cams[i].setRotation(dR * R0);

                     K = _cams[i].getIntrinsic();

                     switch (_mode)
                     {
                        case FULL_BUNDLE_RADIAL_TANGENTIAL:
                        {
                           _distortions[i].p1 += delta[i][11];
                           _distortions[i].p2 += delta[i][12];
                           // No break here!
                        }
                        case FULL_BUNDLE_RADIAL:
                        {
                           _distortions[i].k1 += delta[i][9];
                           _distortions[i].k2 += delta[i][10];
                           // No break here!
                        }
                        case FULL_BUNDLE_FOCAL_LENGTH_PP:
                        {
                           K[0][2] += delta[i][7];
                           K[1][2] += delta[i][8];
                           // No break here!
                        }
                        case FULL_BUNDLE_FOCAL_LENGTH:
                        {
                           double const ar = K[1][1] / K[0][0];
                           K[0][0] += delta[i][6];
                           K[1][1] = ar * K[0][0];
                        }
                        case FULL_BUNDLE_METRIC:
                        {
                        }
                     } // end switch
                     _cams[i].setIntrinsic(K);
                  }
                  break;
               }
               case POINT_PARAM_TYPE:
               {
                  for (int j = 0; j < _Xs.size(); ++j)
                  {
                     _Xs[j][0] += delta[j][0];
                     _Xs[j][1] += delta[j][1];
                     _Xs[j][2] += delta[j][2];
                  }
                  break;
               }
               default:
                  assert(false);
            } // end switch (paramType)
         } // end updateParametersA()

         virtual void saveAllParameters()
         {
            for (int i = 0; i < _cams.size(); ++i)
            {
               _savedTranslations[i] = _cams[i].getTranslation();
               _savedRotations[i]    = _cams[i].getRotation();
               _savedKs[i]           = _cams[i].getIntrinsic();
               _savedDistortions[i]  = _distortions[i];
            }
            _savedXs = _Xs;
         }

         virtual void restoreAllParameters()
         {
            for (int i = 0; i < _cams.size(); ++i)
            {
               _cams[i].setTranslation(_savedTranslations[i]);
               _cams[i].setRotation(_savedRotations[i]);
               _cams[i].setIntrinsic(_savedKs[i]);
               _distortions[i] = _savedDistortions[i];
            }
            _Xs = _savedXs;
         }

      protected:
         int _mode;

         vector<CameraMatrix>&          _cams;
         vector<Vector3d >&             _Xs;
         vector<StdDistortionFunction>& _distortions;

         vector<Vector3d >   _savedTranslations;
         vector<Matrix3x3d > _savedRotations;
         vector<Matrix3x3d>  _savedKs;
         vector<Vector3d >   _savedXs;
         vector<StdDistortionFunction> _savedDistortions;

         double _cachedParamLength;
   }; // end struct VaryingIntrinsicBundleOptimizer

   inline void
   showErrorStatistics(vector<StdDistortionFunction> const& distortions,
                       vector<CameraMatrix> const& cams,
                       vector<Vector3d> const& Xs,
                       vector<Vector2d> const& measurements,
                       vector<int> const& correspondingView,
                       vector<int> const& correspondingPoint)
   {
      int const K = measurements.size();

      double meanReprojectionError = 0.0;
      for (int k = 0; k < K; ++k)
      {
         int const i = correspondingView[k];
         int const j = correspondingPoint[k];
         Vector2d p = cams[i].projectPoint(distortions[i], Xs[j]);

         double const f0 = 1.0; //origFocalLengths[i];
         double reprojectionError = norm_L2(f0 * (p - measurements[k]));
         meanReprojectionError += reprojectionError;
      }
      cout << "mean reprojection error (in pixels): " << meanReprojectionError/K << endl;
   }

} // end namespace <>

int
main(int argc, char * argv[])
{
   if (argc != 3)
   {
      cerr << "Usage: " << argv[0] << " <sparse reconstruction file> <mode>" << endl;
      cout << "<mode> is one of metric, focal, prinipal, radial, tangental." << endl;
      return -1;
   }

   ifstream is(argv[1]);
   if (!is)
   {
      cerr << "Cannot open " << argv[1] << endl;
      return -2;
   }

   int mode = 0;
   if (strcmp(argv[2], "metric") == 0)
      mode = FULL_BUNDLE_METRIC;
   else if (strcmp(argv[2], "focal") == 0)
      mode = FULL_BUNDLE_FOCAL_LENGTH;
   else if (strcmp(argv[2], "principal") == 0)
      mode = FULL_BUNDLE_FOCAL_LENGTH_PP;
   else if (strcmp(argv[2], "radial") == 0)
      mode = FULL_BUNDLE_RADIAL;
   else if (strcmp(argv[2], "tangential") == 0)
      mode = FULL_BUNDLE_RADIAL_TANGENTIAL;
   else
   {
      cerr << "Unknown bundle mode: " << argv[2] << endl;
      return -2;
   }

   int N, M, K;
   is >> M >> N >> K;
   cout << "N (cams) = " << N << " M (points) = " << M << " K (measurements) = " << K << endl;

   vector<StdDistortionFunction> distortions(N);
   vector<double> origFocalLengths(N);
   vector<CameraMatrix> cams(N);

   double avgFocalLength = 0.0;

   for (int i = 0; i < N; ++i)
   {
      Matrix3x3d KMat;
      makeIdentityMatrix(KMat);
      is >> KMat[0][0] >> KMat[0][1] >> KMat[0][2] >> KMat[1][1] >> KMat[1][2]
         >> distortions[i].k1 >> distortions[i].k2 >> distortions[i].p1 >> distortions[i].p2;

      //cout << "K = "; displayMatrix(KMat);

      double const f0 = KMat[0][0];
      Matrix3x3d Knorm = KMat;
      // Normalize the intrinsic to have unit focal length.
      scaleMatrixIP(1.0/f0, Knorm);
      Knorm[2][2] = 1.0;

      origFocalLengths[i] = f0;
      cams[i].setIntrinsic(Knorm);
      avgFocalLength += f0;
   } // end for (i)

   avgFocalLength /= N;
   cout << "mean focal length = " << avgFocalLength << endl;

   vector<int> pointIdFwdMap(M);
   map<int, int> pointIdBwdMap;

   vector<Vector3d > Xs(M);
   for (int j = 0; j < M; ++j)
   {
      int pointId;
      is >> pointId >> Xs[j][0] >> Xs[j][1] >> Xs[j][2];
      pointIdFwdMap[j] = pointId;
      pointIdBwdMap.insert(make_pair(pointId, j));
   }
   cout << "Read the 3D points." << endl;

   vector<int> camIdFwdMap(N);
   map<int, int> camIdBwdMap;

   for (int i = 0; i < N; ++i)
   {
      int camId;
      Matrix3x3d R;
      Vector3d T;

      is >> camId;
      is >> R[0][0] >> R[0][1] >> R[0][2] >> T[0];
      is >> R[1][0] >> R[1][1] >> R[1][2] >> T[1];
      is >> R[2][0] >> R[2][1] >> R[2][2] >> T[2];

      camIdFwdMap[i] = camId;
      camIdBwdMap.insert(make_pair(camId, i));

      cams[i].setRotation(R);
      cams[i].setTranslation(T);
   }
   cout << "Read the cameras." << endl;

   vector<Vector2d > measurements;
   vector<int> correspondingView;
   vector<int> correspondingPoint;

   measurements.reserve(K);
   correspondingView.reserve(K);
   correspondingPoint.reserve(K);

   for (int k = 0; k < K; ++k)
   {
      int view, point;
      Vector3d p, np;

      is >> view >> point;
      is >> p[0] >> p[1] >> p[2];

      if (camIdBwdMap.find(view) != camIdBwdMap.end() &&
          pointIdBwdMap.find(point) != pointIdBwdMap.end())
      {
         int const i = (*camIdBwdMap.find(view)).second;
         double const f0 = origFocalLengths[i];
         // Normalize the measurements to match the unit focal length.
         scaleVectorIP(1.0/f0, p);
         measurements.push_back(Vector2d(p[0], p[1]));
         correspondingView.push_back(camIdBwdMap[view]);
         correspondingPoint.push_back(pointIdBwdMap[point]);
      }
   } // end for (k)

   K = measurements.size();

   cout << "Read " << K << " valid 2D measurements." << endl;

   showErrorStatistics(distortions, cams, Xs, measurements, correspondingView, correspondingPoint);

   V3D::optimizerVerbosenessLevel = 1;
   double const inlierThreshold = 2.0 / avgFocalLength;
   cout << "inlierThreshold = " << inlierThreshold << endl;

   {
      NLSQ_ParamDesc paramDesc;
      paramDesc.nParamTypes = 2;
      paramDesc.dimension[CAMERA_PARAM_TYPE] = 6 + VaryingIntrinsicBundleCostFunction::extParamDimensionFromMode(mode);
      paramDesc.dimension[POINT_PARAM_TYPE]  = 3;

      paramDesc.count[CAMERA_PARAM_TYPE] = cams.size();
      paramDesc.count[POINT_PARAM_TYPE]  = Xs.size();

      vector<int> usedParamTypes;
      usedParamTypes.push_back(CAMERA_PARAM_TYPE);
      usedParamTypes.push_back(POINT_PARAM_TYPE);

      Matrix<int> correspondingParams(measurements.size(), 2);
      for (int k = 0; k < correspondingParams.num_rows(); ++k)
      {
         correspondingParams[k][0] = correspondingView[k];
         correspondingParams[k][1] = correspondingPoint[k];
      }

      VaryingIntrinsicBundleCostFunction costFun(mode, usedParamTypes, inlierThreshold, distortions, cams,
                                                 Xs, measurements, correspondingParams);
      vector<NLSQ_CostFunction *> costFunctions;
      costFunctions.push_back(&costFun);

      VaryingIntrinsicBundleOptimizer opt(mode, paramDesc, costFunctions, distortions, cams, Xs);
      opt.updateThreshold = 1e-8;

      opt.maxIterations = 200;
      opt.minimize();
      cout << "optimizer status = " << opt.status << endl;
   } // end scope

   showErrorStatistics(distortions, cams, Xs, measurements, correspondingView, correspondingPoint);

   {
      ofstream os("refined-nonlinlsq.txt");

      os << M << " " << N << " " << K << endl;

      for (int i = 0; i < N; ++i)
      {
         Matrix3x3d Knew = cams[i].getIntrinsic();
         scaleMatrixIP(origFocalLengths[i], Knew);
         Knew[2][2] = 1.0;
         os << Knew[0][0] << " " << Knew[0][1] << " " << Knew[0][2] << " " << Knew[1][1] << " " << Knew[1][2] << " "
            << distortions[i].k1 << " " << distortions[i].k2 << " "
            << distortions[i].p1 << " " << distortions[i].p2 << " " << endl;
      }

      for (int j = 0; j < M; ++j)
      {
         os << pointIdFwdMap[j] << " " << Xs[j][0] << " " << Xs[j][1] << " " << Xs[j][2] << endl;
      }

      for (int i = 0; i < N; ++i)
      {
         os << camIdFwdMap[i] << " ";
         Matrix3x4d const RT = cams[i].getOrientation();
         os << RT[0][0] << " " << RT[0][1] << " " << RT[0][2] << " " << RT[0][3] << " ";
         os << RT[1][0] << " " << RT[1][1] << " " << RT[1][2] << " " << RT[1][3] << " ";
         os << RT[2][0] << " " << RT[2][1] << " " << RT[2][2] << " " << RT[2][3] << endl;
      }
   } // end scope

   vector<float> norms(Xs.size());

   for (size_t i = 0; i < Xs.size(); ++i)
   {
      Vector3d& X = Xs[i];
      norms[i] = norm_L2(X);
   }
   std::sort(norms.begin(), norms.end());
   float distThr = norms[int(norms.size() * 0.9f)];
   cout << "90% quantile distance: " << distThr << endl;

   for (size_t i = 0; i < Xs.size(); ++i)
   {
      Vector3d& X = Xs[i];
      if (norm_L2(X) > 3*distThr) makeZeroVector(X);
   }

   for (int i = 0; i < cams.size(); ++i)
   {
      cout << "cam[" << i << "]: K = "; displayMatrix(cams[i].getIntrinsic());
      cout << " distortion: k1 = " << distortions[i].k1 << " k2 = " << distortions[i].k2 << endl;
   } // end for (i)

   writePointsToVRML(Xs, "after-ba-nonlinlsq.wrl", false);

   return 0;
}
