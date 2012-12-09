/*****************************************************************************
*   Markerless AR desktop application.
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch3 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/


////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"

Matrix44 Matrix44::getTransposed() const
{
  Matrix44 t;
  
  for (int i=0;i<4; i++)
    for (int j=0;j<4;j++)
      t.mat[i][j] = mat[j][i];
    
  return t;
}

Matrix44 Matrix44::identity()
{
  Matrix44 eye;
  
  for (int i=0;i<4; i++)
    for (int j=0;j<4;j++)
      eye.mat[i][j] = i == j ? 1 : 0;
  
  return eye;
}

Matrix44 Matrix44::getInvertedRT() const
{
  Matrix44 t = identity();
  
  for (int col=0;col<3; col++)
  {
    for (int row=0;row<3;row++)
    { 
      // Transpose rotation component (inversion)
      t.mat[row][col] = mat[col][row];
    }
    
    // Inverse translation component
    t.mat[3][col] = - mat[3][col];
  }
  return t;
}

Matrix33 Matrix33::identity()
{
  Matrix33 eye;
  
  for (int i=0;i<3; i++)
    for (int j=0;j<3;j++)
      eye.mat[i][j] = i == j ? 1 : 0;
  
  return eye;
}

Matrix33 Matrix33::getTransposed() const
{
  Matrix33 t;
  
  for (int i=0;i<3; i++)
    for (int j=0;j<3;j++)
      t.mat[i][j] = mat[j][i];
  
  return t;
}

Vector3 Vector3::zero()
{
  Vector3 v = { 0,0,0 };
  return v;
}

Vector3 Vector3::operator-() const
{
  Vector3 v = { -data[0],-data[1],-data[2] };
  return v;
}

Transformation::Transformation()
: m_rotation(Matrix33::identity())
, m_translation(Vector3::zero())
{
  
}

Transformation::Transformation(const Matrix33& r, const Vector3& t)
: m_rotation(r)
, m_translation(t)
{
  
}

Matrix33& Transformation::r()
{
  return m_rotation;
}

Vector3&  Transformation::t()
{
  return  m_translation;
}

const Matrix33& Transformation::r() const
{
  return m_rotation;
}

const Vector3&  Transformation::t() const
{
  return  m_translation;
}

Matrix44 Transformation::getMat44() const
{
  Matrix44 res = Matrix44::identity();
  
  for (int col=0;col<3;col++)
  {
    for (int row=0;row<3;row++)
    {
      // Copy rotation component
      res.mat[row][col] = m_rotation.mat[row][col];
    }
    
    // Copy translation component
    res.mat[3][col] = m_translation.data[col];
  }
  
  return res;
}

Transformation Transformation::getInverted() const
{
  return Transformation(m_rotation.getTransposed(), -m_translation); 
}