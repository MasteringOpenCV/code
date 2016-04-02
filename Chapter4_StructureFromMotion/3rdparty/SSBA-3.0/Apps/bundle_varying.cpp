// Bundle adjustment application for datasets captured with different cameras (varying intrinsics).

#include "Math/v3d_linear.h"
#include "Geometry/v3d_metricbundle.h"

#include <iostream>
#include <fstream>
#include <map>

using namespace V3D;
using namespace std;

namespace
{

   inline void
   showErrorStatistics(vector<double> const& origFocalLengths,
                       vector<StdDistortionFunction> const& distortions,
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

         double const f0 = origFocalLengths[i];
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

#if 1
      cams[i].setRotation(R);
      cams[i].setTranslation(T);
#else
      cams[i].setRotation(transposedMatrix(R));
      cams[i].setTranslation(transposedMatrix(R) * (-1.0 * T));
#endif
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

   showErrorStatistics(origFocalLengths, distortions, cams, Xs,
                       measurements, correspondingView, correspondingPoint);

   V3D::optimizerVerbosenessLevel = 1;
   double const inlierThreshold = 2.0 / avgFocalLength;
   //double const inlierThreshold = 1000.0;

   Matrix3x3d K0 = cams[0].getIntrinsic();
   //cout << "K0 = "; displayMatrix(K0);

   VaryingInternalsMetricBundleOptimizer opt(mode, inlierThreshold, distortions, cams, Xs,
                                             measurements, correspondingView, correspondingPoint);
   //opt.setNonvaryingCounts(0, Xs.size(), 0);
   opt.maxIterations = 200;
   opt.minimize();
   cout << "optimizer status = " << opt.status << endl;
   cout << "currentIteration = " << opt.currentIteration << endl;

   showErrorStatistics(origFocalLengths, distortions, cams, Xs,
                       measurements, correspondingView, correspondingPoint);

   ofstream os("refined.txt");

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

   return 0;
}
