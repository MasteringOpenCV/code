// -*- C++ -*-
#ifndef V3D_DISTORTION_H
#define V3D_DISTORTION_H

#include "Math/v3d_linear.h"

namespace V3D
{

   struct StdDistortionFunction
   {
         double k1, k2, p1, p2;

         StdDistortionFunction()
            : k1(0), k2(0), p1(0), p2(0)
         { }

         Vector2d operator()(Vector2d const& xu) const
         {
            double const r2 = xu[0]*xu[0] + xu[1]*xu[1];
            double const r4 = r2*r2;
            double const kr = 1 + k1*r2 + k2*r4;

            Vector2d xd;
            xd[0] = kr * xu[0] + 2*p1*xu[0]*xu[1] + p2*(r2 + 2*xu[0]*xu[0]);
            xd[1] = kr * xu[1] + 2*p2*xu[0]*xu[1] + p1*(r2 + 2*xu[1]*xu[1]);
            return xd;
         }

         Matrix2x2d derivativeWrtRadialParameters(Vector2d const& xu) const
         {
            double const r2 = xu[0]*xu[0] + xu[1]*xu[1];
            double const r4 = r2*r2;
            //double const kr = 1 + k1*r2 + k2*r4;

            Matrix2x2d deriv;

            deriv[0][0] = xu[0] * r2; // d xd/d k1
            deriv[0][1] = xu[0] * r4; // d xd/d k2
            deriv[1][0] = xu[1] * r2; // d yd/d k1
            deriv[1][1] = xu[1] * r4; // d yd/d k2
            return deriv;
         }

         Matrix2x2d derivativeWrtTangentialParameters(Vector2d const& xu) const
         {
            double const r2 = xu[0]*xu[0] + xu[1]*xu[1];
            //double const r4 = r2*r2;
            //double const kr = 1 + k1*r2 + k2*r4;

            Matrix2x2d deriv;
            deriv[0][0] = 2*xu[0]*xu[1];      // d xd/d p1
            deriv[0][1] = r2 + 2*xu[0]*xu[0]; // d xd/d p2
            deriv[1][0] = r2 + 2*xu[1]*xu[1]; // d yd/d p1
            deriv[1][1] = deriv[0][0];        // d yd/d p2
            return deriv;
         }

         Matrix2x2d derivativeWrtUndistortedPoint(Vector2d const& xu) const
         {
            double const r2 = xu[0]*xu[0] + xu[1]*xu[1];
            double const r4 = r2*r2;
            double const kr = 1 + k1*r2 + k2*r4;
            double const dkr = 2*k1 + 4*k2*r2;

            Matrix2x2d deriv;
            deriv[0][0] = kr + xu[0] * xu[0] * dkr + 2*p1*xu[1] + 6*p2*xu[0]; // d xd/d xu
            deriv[0][1] =      xu[0] * xu[1] * dkr + 2*p1*xu[0] + 2*p2*xu[1]; // d xd/d yu
            deriv[1][0] = deriv[0][1];                                        // d yd/d xu
            deriv[1][1] = kr + xu[1] * xu[1] * dkr + 6*p1*xu[1] + 2*p2*xu[0]; // d yd/d yu
            return deriv;
         }
   }; // end struct StdDistortionFunction

   struct StdPixelDistortionFunction
   {
         double k1, k2, p1, p2;
         Vector2d center;

         StdPixelDistortionFunction()
            : k1(0), k2(0), p1(0), p2(0)
         {
            makeZeroVector(center);
         }

         Vector2d operator()(Vector2d const& xu) const
         {
            Vector2d const np = xu - center;
            double const r2 = np[0]*np[0] + np[1]*np[1];
            double const r4 = r2*r2;
            double const kr = 1 + k1*r2 + k2*r4;

            Vector2d xd;
            xd[0] = kr * np[0] + center[0] + 2*p1*np[0]*np[1] + p2*(r2 + 2*np[0]*np[0]);
            xd[1] = kr * np[1] + center[1] + 2*p2*np[0]*np[1] + p1*(r2 + 2*np[1]*np[1]);
            return xd;
         }

         Matrix2x2d derivativeWrtRadialParameters(Vector2d const& xu) const
         {
            Vector2d const np = xu - center;
            double const r2 = np[0]*np[0] + np[1]*np[1];
            double const r4 = r2*r2;
            //double const kr = 1 + k1*r2 + k2*r4;

            Matrix2x2d deriv;

            deriv[0][0] = np[0] * r2; // d xd/d k1
            deriv[0][1] = np[0] * r4; // d xd/d k2
            deriv[1][0] = np[1] * r2; // d yd/d k1
            deriv[1][1] = np[1] * r4; // d yd/d k2
            return deriv;
         }

         Matrix2x2d derivativeWrtTangentialParameters(Vector2d const& xu) const
         {
            Vector2d const np = xu - center;
            double const r2 = np[0]*np[0] + np[1]*np[1];
            //double const r4 = r2*r2;
            //double const kr = 1 + k1*r2 + k2*r4;

            Matrix2x2d deriv;
            deriv[0][0] = 2*np[0]*np[1];      // d xd/d p1
            deriv[0][1] = r2 + 2*np[0]*np[0]; // d xd/d p2
            deriv[1][0] = r2 + 2*np[1]*np[1]; // d yd/d p1
            deriv[1][1] = deriv[0][0];        // d yd/d p2
            return deriv;
         }

         Matrix2x2d derivativeWrtCenter(Vector2d const& xu) const
         {
            Vector2d const np = xu - center;
            double const r2 = np[0]*np[0] + np[1]*np[1];
            double const r4 = r2*r2;
            double const kr = 1 + k1*r2 + k2*r4;
            double const dkr = 2*k1 + 4*k2*r2;

            Matrix2x2d deriv;
            deriv[0][0] = 1 - kr - np[0]*np[0] * dkr - 2*p1*np[1] - 6*p2*np[0]; // d xd/d cx
            deriv[0][1] =        - np[0]*np[1] * dkr - 2*p1*np[0] - 2*p2*np[1]; // d xd/d cy
            deriv[1][0] = deriv[0][1];                                          // d yd/d cx
            deriv[1][1] = 1 - kr - np[1]*np[1] * dkr - 6*p1*np[1] - 2*p2*np[0]; // d yd/d cy
            return deriv;
         }

         Matrix2x2d derivativeWrtUndistortedPoint(Vector2d const& xu) const
         {
            Vector2d const np = xu - center;
            double const r2 = np[0]*np[0] + np[1]*np[1];
            double const r4 = r2*r2;
            double const kr = 1 + k1*r2 + k2*r4;
            double const dkr = 2*k1 + 4*k2*r2;

            Matrix2x2d deriv;
            deriv[0][0] = kr + np[0] * np[0] * dkr + 2*p1*np[1] + 6*p2*np[0]; // d xd/d xu
            deriv[0][1] =      np[0] * np[1] * dkr + 2*p1*np[0] + 2*p2*np[1]; // d xd/d yu
            deriv[1][0] = deriv[0][1];                                        // d yd/d xu
            deriv[1][1] = kr + np[1] * np[1] * dkr + 6*p1*np[1] + 2*p2*np[0]; // d yd/d yu
            return deriv;
         }
   }; // end struct StdPixelDistortionFunction

} // end namespace V3D

#endif
