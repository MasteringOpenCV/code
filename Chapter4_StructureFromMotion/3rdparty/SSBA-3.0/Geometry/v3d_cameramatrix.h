// -*- C++ -*-

#ifndef V3D_CAMERA_MATRIX_H
#define V3D_CAMERA_MATRIX_H

#include "Base/v3d_serialization.h"
#include "Math/v3d_linear.h"
#include "Geometry/v3d_distortion.h"

namespace V3D
{

   struct CameraMatrix
   {
         CameraMatrix()
         {
            makeIdentityMatrix(_K);
            makeIdentityMatrix(_R);
            makeZeroVector(_T);
            this->updateCachedValues(true, true);
         }

         CameraMatrix(double f, double cx, double cy)
         {
            makeIdentityMatrix(_K);
            _K[0][0] = f;
            _K[1][1] = f;
            _K[0][2] = cx;
            _K[1][2] = cy;
            makeIdentityMatrix(_R);
            makeZeroVector(_T);
            this->updateCachedValues(true, true);
         }

         CameraMatrix(Matrix3x3d const& K,
                      Matrix3x3d const& R,
                      Vector3d const& T)
            : _K(K), _R(R), _T(T)
         {
            this->updateCachedValues(true, true);
         }

         CameraMatrix(Matrix3x3d const& K,
                      Matrix3x4d const& RT)
            : _K(K)
         {
             _R = RT.slice<3,3>(0,0);
             _T = RT.col(3);
            this->updateCachedValues(true, true);
         }

         template<typename T> T sqr(T t) { return t*t; }

         CameraMatrix(Matrix3x4d const& P)
         {
            Matrix3x3d K;
            copyMatrixSlice(P, 0, 0, 3, 3, K, 0, 0);

            // get affine matrix (rq-decomposition of M)
            // See Hartley & Zissermann, p552 (1st ed.)
            double h = sqrt(sqr(K[2][1]) + sqr(K[2][2]));
            double s =  K[2][1] / h;
            double c = -K[2][2] / h;

            Matrix3x3d Rx; makeZeroMatrix(Rx);
            Rx[0][0] =  1;
            Rx[1][1] =  c; Rx[2][2] = c;
            Rx[1][2] = -s; Rx[2][1] = s;

            K = K * Rx;

            h = sqrt(sqr(K[2][0]) + sqr(K[2][2]));
            s =  K[2][0] / h;
            c = -K[2][2] / h;

            Matrix3x3d Ry; makeZeroMatrix(Ry);
            Ry[1][1] =  1;
            Ry[0][0] =  c; Ry[2][2] = c;
            Ry[0][2] = -s; Ry[2][0] = s;

            K = K * Ry;

            h = sqrt(sqr(K[1][0]) + sqr(K[1][1]));
            s =  K[1][0] / h;
            c = -K[1][1] / h;

            Matrix3x3d Rz; makeZeroMatrix(Rz);
            Rz[2][2] =  1;
            Rz[0][0] =  c; Rz[1][1] = c;
            Rz[0][1] = -s; Rz[1][0] = s;

            K = K * Rz;

            Matrix3x3d Sign;
            makeIdentityMatrix(Sign);

            if (K[0][0] < 0) Sign[0][0] = -1;
            if (K[1][1] < 0) Sign[1][1] = -1;
            if (K[2][2] < 0) Sign[2][2] = -1;

            K = K * Sign; // Change signum of columns

            Matrix3x3d R = Rx * Ry * Rz * Sign;
            transposeMatrixIP(R);

            Vector3d P4;
            P.getColumnSlice(0, 3, 3, P4);
            Vector3d T = invertedMatrix(K) * P4;

            scaleMatrixIP(1.0 / K[2][2], K); // Normalize, such that lower-right element is 1.

            _K = K;
            _R = R;
            _T = T;
            this->updateCachedValues(true, true);
         }

         CameraMatrix & operator = ( const CameraMatrix &cam )
         {
             _K = cam.getIntrinsic();
             _R = cam.getRotation();
             _T = cam.getTranslation();
             _size=cam._size;
             this->updateCachedValues(true, true);
             return *this;
         }

         void setIntrinsic(Matrix3x3d const& K) { _K = K; this->updateCachedValues(true, false); }
         void setRotation(Matrix3x3d const& R) { _R = R; this->updateCachedValues(false, true); }
         void setTranslation(Vector3d const& T) { _T = T; this->updateCachedValues(false, true); }

         void setCameraCenter(Vector3d const& c) { this->setTranslation(-1.0 * (_R * c)); }

         template <typename Mat>
         void setOrientation(Mat const& RT)
         {
            _R[0][0] = RT[0][0]; _R[0][1] = RT[0][1]; _R[0][2] = RT[0][2];
            _R[1][0] = RT[1][0]; _R[1][1] = RT[1][1]; _R[1][2] = RT[1][2];
            _R[2][0] = RT[2][0]; _R[2][1] = RT[2][1]; _R[2][2] = RT[2][2];
            _T[0]    = RT[0][3]; _T[1]    = RT[1][3]; _T[2]    = RT[2][3];
            this->updateCachedValues(false, true);
         }

         Matrix3x3d const& getIntrinsic()   const { return _K; }
         Matrix3x3d const& getRotation()    const { return _R; }
         Vector3d   const& getTranslation() const { return _T; }

         Matrix3x4d getExtrinsic() const
         {
            Matrix3x4d RT;
            copyMatrixSlice(_R,0,0,3,3,RT,0,0);
            RT[0][3] = _T[0];
            RT[1][3] = _T[1];
            RT[2][3] = _T[2];
            return RT;
         }

         Matrix3x4d getOrientation() const
         {
            Matrix3x4d RT;
            RT[0][0] = _R[0][0]; RT[0][1] = _R[0][1]; RT[0][2] = _R[0][2];
            RT[1][0] = _R[1][0]; RT[1][1] = _R[1][1]; RT[1][2] = _R[1][2];
            RT[2][0] = _R[2][0]; RT[2][1] = _R[2][1]; RT[2][2] = _R[2][2];
            RT[0][3] = _T[0];    RT[1][3] = _T[1];    RT[2][3] = _T[2];
            return RT;
         }

         Matrix3x4d getProjection() const
         {
            Matrix3x4d const RT = this->getOrientation();
            return _K * RT;
         }

         double getFocalLength() const { return _K[0][0]; }
         double getAspectRatio() const { return _K[1][1] / _K[0][0]; }

         Vector2d getPrincipalPoint() const
         {
            Vector2d pp;
            pp[0] = _K[0][2];
            pp[1] = _K[1][2];
            return pp;
         }

         Vector2d projectPoint(Vector3d const& X) const
         {
            Vector3d q = _K*(_R*X + _T);
            Vector2d res;
            res[0] = q[0]/q[2]; res[1] = q[1]/q[2];
            return res;
         }

         template <typename Distortion>
         Vector2d projectPoint(Distortion const& distortion, Vector3d const& X) const
         {
            Vector3d XX = _R*X + _T;
            Vector2d p;
            p[0] = XX[0] / XX[2];
            p[1] = XX[1] / XX[2];
            p = distortion(p);

            Vector2d res;
            res[0] = _K[0][0] * p[0] + _K[0][1] * p[1] + _K[0][2];
            res[1] =                   _K[1][1] * p[1] + _K[1][2];
            return res;
         }
         template<typename T>
         Vector3d unprojectPixel(InlineVector<T,2> const &p, double depth = 1) const
         {
            Vector3d pp;
            pp[0] = p[0]; pp[1] = p[1]; pp[2] = 1.0;
            Vector3d ray = _invK * pp;
            ray[0] *= depth/ray[2];
            ray[1] *= depth/ray[2];
            ray[2] = depth;
            ray = _Rt * ray;
            return _center + ray;
         }

         Vector3d intersectRayWithPlane( const Vector4d &plane, int x, int y ) const
         {
             Vector3d ray = getRay(Vector2d(x,y));
             double rho = (-innerProduct(plane.slice<3>(0),_center) - plane[3]) /
                 innerProduct(plane.slice<3>(0),ray);
             return _center + rho*ray;
         }

         Vector3d transformPointIntoCameraSpace(Vector3d const& p) const
         {
            return _R*p + _T;
         }

         Vector3d transformPointFromCameraSpace(Vector3d const& p) const
         {
            return _Rt*(p-_T);
         }

         Vector3d transformDirectionFromCameraSpace(Vector3d const& dir) const
         {
            return _Rt*dir;
         }

         Vector3d transformDirectionIntoCameraSpace(Vector3d const& dir) const
         {
            return _R*dir;
         }
         template<typename T>
         InlineVector<T,2> transformPointIntoNormalizedCoordinate(InlineVector<T,2> const& p) const
         {
            return InlineVector<T,2>(_K[0][0] * p[0] + _K[0][1] * p[1] + _K[0][2],_K[1][1] * p[1] + _K[1][2]);
         }
         template<typename T>
         InlineVector<T,2> transformPointFromNormalizedCoordinate(InlineVector<T,2> const& p) const
         {
            return InlineVector<T,2>(_invK[0][0] * p[0] + _invK[0][1] * p[1] + _invK[0][2],_invK[1][1] * p[1] + _invK[1][2]);
         }
         Vector3d const& cameraCenter() const
         {
            return _center;
         }

         Vector3d opticalAxis() const
         {
            return this->transformDirectionFromCameraSpace(Vector3d(0.0, 0.0, 1.0));
         }

         Vector3d upVector() const
         {
            return this->transformDirectionFromCameraSpace(Vector3d(0.0, 1.0, 0.0));
         }

         Vector3d rightVector() const
         {
            return this->transformDirectionFromCameraSpace(Vector3d(1.0, 0.0, 0.0));
         }

         template <typename Vec2>
         Vector3d getRay(Vec2 const& p) const
         {
            Vector3d pp = Vector3d(p[0], p[1], 1.0);
            Vector3d ray = _invK * pp;
            ray = _Rt * ray;
            normalizeVector(ray);
            return ray;
         }

         template <typename Vec2>
         Vector3d getCameraRay(Vec2 const& p) const
         {
            return _invK * Vector3d(p[0], p[1], 1.0);
         }

         template <typename Vec3>
         inline bool isOnGoodSide(Vec3 const &p) const
         {
            return transformPointIntoCameraSpace(p)[2]>0;
         }
         template <typename Archive>
         void serialize(Archive& ar)
         {
            V3D::SerializationScope<Archive> scope(ar, "CameraMatrix");
            ar & _K & _R & _T;
            if (ar.isLoading())
               this->updateCachedValues(true, true);
         }

         float getWidth() const
         {
            return _size[0];
         }
         float getHeight() const
         {
            return _size[1];
         }
         void setSize(float w,float h)
         {
            _size[0]=w;
            _size[1]=h;
         }
         V3D_DEFINE_LOAD_SAVE(CameraMatrix)

      protected:
         void updateCachedValues(bool intrinsic, bool orientation)
         {
            if (intrinsic) _invK = invertedMatrix(_K);

            if (orientation)
            {
               makeTransposedMatrix(_R, _Rt);
               _center = _Rt * (-1.0 * _T);
            }
         }

         Matrix3x3d _K, _R;
         Vector3d   _T;
         Matrix3x3d _invK, _Rt;
         Vector3d   _center;
         Vector2f _size;
   }; // end struct CameraMatrix

   V3D_DEFINE_IOSTREAM_OPS(CameraMatrix)

} // end namespace V3D

#endif
