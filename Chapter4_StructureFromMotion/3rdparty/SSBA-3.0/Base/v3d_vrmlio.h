#ifndef V3D_VRMLIO_H
#define V3D_VRMLIO_H

#include "Geometry/v3d_cameramatrix.h"
#include <iostream>
#include <string>
#include <cstdio>

namespace V3D
{

   template <typename T,typename U>
   void writeColoredPointsToVRML(std::vector<T> const& points, std::vector<U> const& colors255,
                                 char const * filename, bool append = false)
   {
      using namespace std;
      std::ofstream os;

      os.open(filename, append ? ios::app : ios::out);
      os.precision(15);

      if (!append) os <<"#VRML V2.0 utf8" << endl;
      os << "Shape {" << endl;

      os << " appearance Appearance {" << endl;
      os << "   material Material {" << endl;
      os << "        ambientIntensity 0.1" << endl;
      os << "        diffuseColor 1.0 0.7 0.2" << endl;
      os << " } } " << endl;
      os << "  geometry PointSet {" << endl;

      // write point coordinates
      os << "  coord Coordinate {" << endl;
      os << "  point [" << endl;
      for (size_t i = 0; i < points.size(); ++i)
         os << points[i][0] <<" " << points[i][1] << " " << points[i][2] << "," << endl;
      os << " ]\n } # end points"<<endl;

      os << " color Color { color [" << endl;
      for (size_t i = 0; i < points.size(); ++i)
      {
         double r = ((double) colors255[i][0]) / 255.0;
         double g = ((double) colors255[i][1]) / 255.0;
         double b = ((double) colors255[i][2]) / 255.0;
         os << min(max(r, 0.0), 1.0) << " " << min(max(g, 0.0), 1.0) << " " << min(max(b, 0.0), 1.0) << " ," << endl;
      }
      os << " ] } # end color" << endl;
      os << " }" << endl;
      os << "}" << endl;
   } // end writeColoredPointsToVRML()

   template <typename U>
   void writeCameraFrustumToVRML(CameraMatrix const& cam, int width, int height, double scale,
                                 U const& color255, const char *filename, bool append)
   {
      using namespace std;

      ofstream os;

      os.open(filename, append ? ios::app : ios::out);
      os.precision(15);

      double r = ((double) color255[0]) / 255.0;
      double g = ((double) color255[1]) / 255.0;
      double b = ((double) color255[2]) / 255.0;

      if (!append) os <<"#VRML V2.0 utf8" << endl;
      os << " Shape {" << endl;
      os << " appearance Appearance {" << endl;
      os << "  material Material { emissiveColor 0.2 0.2 0.2" << endl;
      os << "  diffuseColor " << r << " " << g << " " << b << endl;
      os << "  }" << endl;
      os << " }" << endl;

      os <<" geometry IndexedFaceSet {" << endl;
      os <<" solid FALSE " << endl;
      os <<" ccw TRUE " << endl;
      os <<" coord Coordinate {" << endl;
      os <<" point [" << endl;

      Vector3d const center = cam.cameraCenter();
      Vector3d const ll = cam.unprojectPixel(makeVector2(0.0, double(height)), scale);
      Vector3d const lr = cam.unprojectPixel(makeVector2(double(width), double(height)), scale);
      Vector3d const ul = cam.unprojectPixel(makeVector2(0.0, 0.0), scale);
      Vector3d const ur = cam.unprojectPixel(makeVector2(double(width), 0.0), scale);

      // write point coordinates
      os << ll[0] << " " << ll[1] << " " << ll[2] << "," << endl;
      os << lr[0] << " " << lr[1] << " " << lr[2] << "," << endl;
      os << ul[0] << " " << ul[1] << " " << ul[2] << "," << endl;
      os << ur[0] << " " << ur[1] << " " << ur[2] << "," << endl;
      os << center[0] << " " << center[1] << " " << center[2] << "," << endl;

      os << " ]\n } "<< endl;

      os << "  coordIndex [\n";
      os << "  0,1,2,3,-1" << endl;
      os << "  0,1,4,-1" << endl;
      os << "  1,3,4,-1" << endl;
      os << "  3,2,4,-1" << endl;
      os << "  2,0,4,-1\n  ]" << endl;
      os << " }\n}" << endl;
   } // end writeCameraFrustumToVRML()

   template <typename T>
   void writeMatchesToVRML(std::vector<T> const& points, char const * filename, bool append = false)
   {
       using namespace std;
       std::ofstream os;

       os.open(filename, append ? ios::app : ios::out);
       os.precision(15);

       os << "#VRML V2.0 utf8" << endl;
       os << " Shape {" << endl;

       os << " appearance Appearance {" << endl;
       os << " material Material {" << endl;
       os << "     ambientIntensity 0.1" << endl;
       os << "     diffuseColor 1.0 0.0 0.0" << endl;
       os << "     transparency      0.5" << endl;
       os << " } } " << endl;
       os << "  geometry IndexedLineSet {" << endl;

       // write lines between matches coordinates
       os <<"  coord Coordinate {" << endl;
       os <<"  point [" << endl;
       for (size_t i = 0; i < points.size(); ++i)
           os << points[i][0] <<" " << points[i][1] << " " << points[i][2] << "," << endl;
       os << " ]\n } # end coord"<<endl;

       os <<"  coordIndex ["<<endl;
       for(size_t i=0; i<points.size()-1; i+=2)
           os <<i<<", "<<i+1<<", -1,"<<endl;
       os <<"  ] #end coordIndex"<<endl;

       os << " } # end geometry" << endl;
       os << "} # end shape" << endl;
   }

   template <typename T,typename U>
   void writePointsToVRML(std::vector<T> const& points, U const & color255,char const * filename, bool append = false)
   {
      using namespace std;
      std::ofstream os;

      os.open(filename, append ? ios::app : ios::out);
      os.precision(4);// Precision for color

      os << "#VRML V2.0 utf8" << endl;
      os << " Shape {" << endl;
      writeAppearanceToVRMLStream(os,color255);

      os << "  geometry PointSet {" << endl;
      os.precision(15);// Precision for points

      // write point coordinates
      os <<"  coord Coordinate {" << endl;
      os <<"  point [" << endl;
      for (size_t i = 0; i < points.size(); ++i)
         os << points[i][0] <<" " << points[i][1] << " " << points[i][2] << "," << endl;
      os << " ]\n } # end coord" << endl;

      os << " } # end geometry" << endl;
      os << "} # end shape" << endl;
   }

   template <typename T>
   void writePointsToVRML(std::vector<T> const& points, char const * filename, bool append = false)
   {
      Vector3f color255(makeVector3(255.0,180.0,50.0));
      writePointsToVRML(points, color255, filename, append);
   }

   template <typename U>
   void writeCameraFrustumToVRMLStream(ofstream & os,CameraMatrix const& cam,  double scale,
                                  U const& color255)
    {
       using namespace std;

       os.precision(3);
       os << " Shape {" << endl;
       writeAppearanceToVRMLStream(os,color255);

       os.precision(15);
       os <<" geometry IndexedFaceSet {" << endl;
       os <<" solid FALSE " << endl;
       os <<" ccw TRUE " << endl;
       os <<" coord Coordinate {" << endl;
       os <<" point [" << endl;
       Vector3d const center = cam.cameraCenter();
       Vector3d ll = cam.unprojectPixel(makeVector2(0.0, (double)cam.getHeight()), scale);
       Vector3d lr = cam.unprojectPixel(makeVector2((double)cam.getWidth(), (double)cam.getHeight()), scale);
       Vector3d ul = cam.unprojectPixel(makeVector2(0.0, 0.0), scale);
       Vector3d ur = cam.unprojectPixel(makeVector2((double)cam.getWidth(), 0.0), scale);
       Matrix3x3d I;makeIdentityMatrix(I);
       if(cam.getIntrinsic()==I ){//If normalized Camera, assume 4/3 AspectRatio for Frustrum
          ll = cam.unprojectPixel(makeVector2(-0.5,  0.375), scale);
          lr = cam.unprojectPixel(makeVector2( 0.5,  0.375), scale);
          ul = cam.unprojectPixel(makeVector2(-0.5, -0.375), scale);
          ur = cam.unprojectPixel(makeVector2( 0.5, -0.375), scale);
       }
       // write point coordinates
       os << ll[0] << " " << ll[1] << " " << ll[2] << "," << endl;
       os << lr[0] << " " << lr[1] << " " << lr[2] << "," << endl;
       os << ul[0] << " " << ul[1] << " " << ul[2] << "," << endl;
       os << ur[0] << " " << ur[1] << " " << ur[2] << "," << endl;
       os << center[0] << " " << center[1] << " " << center[2] << "," << endl;

       os << " ]\n } "<< endl;

       os << "  coordIndex [\n";
       //os << "  0,1,2,3,-1" << endl;
       os << "  0,1,4,-1" << endl;
       os << "  1,3,4,-1" << endl;
       os << "  3,2,4,-1" << endl;
       os << "  2,0,4,-1\n  ]" << endl;
       os << " }\n}" << endl;
    } // end writeCameraFrustumToVRML()
   template <typename U>
    void writeCameraFrustumToVRML(CameraMatrix const& cam, double scale,
                                   U const& color255, const char *filename, bool append)
     {
        using namespace std;
        ofstream os;
        os.open(filename, append ? ios::app : ios::out);
        os.precision(3);
        if (!append) os <<"#VRML V2.0 utf8" << endl;
        writeCameraFrustumToVRMLStream(os,cam,  scale, color255);
     } // end writeCameraFrustumToVRML()

    template<typename U>
    void writeAppearanceToVRMLStream(ofstream & os,U const& color255)
    {
       double r = ((double) color255[0]) / 255.0;
       double g = ((double) color255[1]) / 255.0;
       double b = ((double) color255[2]) / 255.0;
       os << " appearance Appearance {" << endl;
       os << "  material Material { ambientIntensity 0.1 " << endl;
       os << "  diffuseColor " << r << " " << g << " " << b << endl;
       os << "  transparency      0.5" << endl;
       os << "  }" << endl;
       os << " }" << endl;
    }

    template <typename U,typename T,typename V>
    void write2DnormalizedVs3DMatchesToVRMLStream(ofstream & os,CameraMatrix const& cam,  std::vector<InlineVector<T,2> > const& points2D,std::vector<InlineVector<V,3> > const& points3D,double scale,
                                   vector<InlineVector<U,3> > const& colors255)
     {
        using namespace std;
        os.precision(15);
        assert(points3D.size()==points2D.size());
        assert(colors255.size()==points2D.size());
        os << " Shape {" << endl;
        Vector3b color255(255,255,255);
        writeAppearanceToVRMLStream(os,color255);
        os << "  geometry IndexedFaceSet {" << endl;

        // write lines between matches coordinates
        os <<"  coord Coordinate {" << endl;
        os <<"  point [" << endl;
        os << cam.cameraCenter()[0] <<" " << cam.cameraCenter()[1] << " " << cam.cameraCenter()[2] << "," << endl;
        for (size_t i = 0; i < points2D.size(); ++i){
           Vector3d p2=cam.unprojectPixel(points2D[i],scale);
           os << p2[0] <<" " << p2[1] << " " << p2[2] << "," << endl;
           os << p2[0] <<" " << p2[1] << " " << p2[2] << "," << endl;
           os << points3D[i][0] <<" " << points3D[i][1] << " " << points3D[i][2] << "," << endl;
        }
        os << " ]\n } # end coord"<<endl;

        os <<"  coordIndex ["<<endl;
        for(size_t i=0; i<points2D.size(); i++){
           os <<0<<", "<<3*i+1<<", "<<3*i+2<<", -1,"<<endl;
           os <<3*i+1<<", "<<3*i+2<<", "<<3*i+3<<", -1,"<<endl;
        }
        os <<"  ] #end coordIndex"<<endl;

        os << " colorPerVertex TRUE" << endl;
        os << " color Color { color [" << endl;
        os << 0 << " " << 0 << " " << 0 << " ," << endl;
        for (size_t i = 0; i < points2D.size(); ++i)
        {
           double r = min(max(((double) colors255[i][0]) / 255.0, 0.0), 1.0);
           double g = min(max(((double) colors255[i][1]) / 255.0, 0.0), 1.0);
           double b = min(max(((double) colors255[i][2]) / 255.0, 0.0), 1.0);
           os << r << " " << g << " " << b << " ," << endl;
           os << r << " " << g << " " << b << " ," << endl;
           os << r << " " << g << " " << b << " ," << endl;
        }
        os << " ] } # end Color" << endl;
        os << " } # end geometry" << endl;
        os << "} # end shape" << endl;

     } // end writeCamera3D2DMatchesToVRMLStream()
    template <typename U,typename T,typename V>
    void write2DnormalizedVs3DMatchesToVRMLStream(ofstream & os,CameraMatrix const& cam,  std::vector<InlineVector<T,2> > const& points2D,std::vector<InlineVector<V,3> > const& points3D,double scale,
                                   InlineVector<U,3> const& color255)
     {
        using namespace std;
        os.precision(15);

        os << " Shape {" << endl;
        writeAppearanceToVRMLStream(os,color255);
        os << "  geometry IndexedFaceSet {" << endl;

        // write lines between matches coordinates
        os <<"  coord Coordinate {" << endl;
        os <<"  point [" << endl;
        os << cam.cameraCenter()[0] <<" " << cam.cameraCenter()[1] << " " << cam.cameraCenter()[2] << "," << endl;
        for (size_t i = 0; i < points2D.size(); ++i){
           Vector3d p2=cam.unprojectPixel(points2D[i],scale);
           os << p2[0] <<" " << p2[1] << " " << p2[2] << "," << endl;
           os << p2[0] <<" " << p2[1] << " " << p2[2] << "," << endl;
           os << points3D[i][0] <<" " << points3D[i][1] << " " << points3D[i][2] << "," << endl;
        }
        os << " ]\n } # end coord"<<endl;

        os <<"  coordIndex ["<<endl;
        for(size_t i=0; i<points2D.size(); i++){
           os <<0<<", "<<3*i+1<<", "<<3*i+2<<", -1,"<<endl;
           os <<3*i+1<<", "<<3*i+2<<", "<<3*i+3<<", -1,"<<endl;
        }
        os <<"  ] #end coordIndex"<<endl;

        os << " } # end geometry" << endl;
        os << "} # end shape" << endl;

     } // end writeCamera3D2DMatchesToVRMLStream()

   template <typename U,typename T,typename V>
   void write2DnormalizedVs3DMatchesToVRML(CameraMatrix const& cam,  std::vector<InlineVector<T,2> > const& points2D,std::vector<InlineVector<V,3> > const& points3D,double scale,
                                  U const& color255,const char *filename, bool append)
   {
      using namespace std;
      ofstream os;
      os.open(filename, append ? ios::app : ios::out);
      os.precision(3);
      if (!append) os <<"#VRML V2.0 utf8" << endl;
      write2DnormalizedVs3DMatchesToVRMLStream(os, cam, points2D, points3D, scale, color255);
   }

}

#endif // V3D_VRMLIO_H
