/*****************************************************************************
*   GeometryTypes.hpp
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#ifndef Example_MarkerBasedAR_GeometryTypes_hpp
#define Example_MarkerBasedAR_GeometryTypes_hpp

struct Matrix44
{
  union
  {
    float data[16];
    float mat[4][4];
  };
  
  Matrix44 getTransposed() const;
  Matrix44 getInvertedRT() const;
  static Matrix44 identity();
};

struct Matrix33
{
  union
  {
    float data[9];
    float mat[3][3];
  };
  
  static Matrix33 identity();
  Matrix33 getTransposed() const;
};

struct Vector4
{
  float data[4];
};

struct Vector3
{
  float data[3];
  
  static Vector3 zero();
  Vector3 operator-() const;
};

struct Transformation
{
  Transformation();
  Transformation(const Matrix33& r, const Vector3& t);
  
  Matrix33& r();
  Vector3&  t();
  
  const Matrix33& r() const;
  const Vector3&  t() const;
  
  Matrix44 getMat44() const;
  
  Transformation getInverted() const;
private:
  Matrix33 m_rotation;
  Vector3  m_translation;
};

#endif
