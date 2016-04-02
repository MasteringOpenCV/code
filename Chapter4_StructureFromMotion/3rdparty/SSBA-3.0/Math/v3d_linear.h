// -*- C++ -*-

#ifndef V3D_LINEAR_H
#define V3D_LINEAR_H

#include "Math/v3d_linearbase.h"

#include <iostream>

namespace V3D
{

   template <typename Elem, int Size>
   struct InlineVector : public InlineVectorBase<Elem, Size>
   {
         InlineVector() { }

         template<typename Elem2>
         explicit InlineVector<Elem,Size>(InlineVector<Elem2,Size> const& a)
         {
            for(int i=0; i<Size; i++)
               this->_vec[i] = (Elem2)a[i];
         }

         template <int size>
         InlineVector<Elem,size> slice( int first ) const
         {
            InlineVector<Elem,size> vec;
            copyVectorSlice(*this,first,size,vec,0);
            return vec;
         }
   }; // end struct InlineVector

   template <typename Elem>
   struct InlineVector<Elem, 2> : public InlineVectorBase<Elem, 2>
   {
         InlineVector() { }

         InlineVector(Elem a, Elem b)
         {
            InlineVectorBase<Elem, 2>::_vec[0] = a;
            InlineVectorBase<Elem, 2>::_vec[1] = b;
         }

         template<typename Elem2>
         explicit InlineVector(InlineVector<Elem2,2> const& a)
         {
            for(int i=0; i<2; i++)
               this->_vec[i] = (Elem2)a[i];
         }

       template <int size>
       InlineVector<Elem,size> slice( int first ) const
       {
           InlineVector<Elem,size> vec;
           copyVectorSlice(*this,first,size,vec,0);
           return vec;
       }
   }; // end struct InlineVector

   template <typename Elem>
   struct InlineVector<Elem, 3> : public InlineVectorBase<Elem, 3>
   {
         InlineVector() { }

         InlineVector(Elem a, Elem b, Elem c)
         {
            InlineVectorBase<Elem, 3>::_vec[0] = a;
            InlineVectorBase<Elem, 3>::_vec[1] = b;
            InlineVectorBase<Elem, 3>::_vec[2] = c;
         }

         InlineVector(InlineVector<Elem,2> const &vec, Elem c)
         {
            InlineVectorBase<Elem, 3>::_vec[0] = vec[0];
            InlineVectorBase<Elem, 3>::_vec[1] = vec[1];
            InlineVectorBase<Elem, 3>::_vec[2] = c;
         }

         template<typename Elem2>
         explicit InlineVector(InlineVector<Elem2,3> const& a)
         {
            for(int i=0; i<3; i++)
               this->_vec[i] = (Elem2)a[i];
         }

       template <int size>
       InlineVector<Elem,size> slice( int first ) const
       {
           InlineVector<Elem,size> vec;
           copyVectorSlice(*this,first,size,vec,0);
           return vec;
       }
   }; // end struct InlineVector

   template <typename Elem>
   struct InlineVector<Elem, 4> : public InlineVectorBase<Elem, 4>
   {
         InlineVector() { }

         InlineVector(Elem a, Elem b, Elem c, Elem d)
         {
            InlineVectorBase<Elem, 4>::_vec[0] = a;
            InlineVectorBase<Elem, 4>::_vec[1] = b;
            InlineVectorBase<Elem, 4>::_vec[2] = c;
            InlineVectorBase<Elem, 4>::_vec[3] = d;
         }

         InlineVector( InlineVector<Elem,3> const& v, Elem d )
         {
            InlineVectorBase<Elem, 4>::_vec[0] = v[0];
            InlineVectorBase<Elem, 4>::_vec[1] = v[1];
            InlineVectorBase<Elem, 4>::_vec[2] = v[2];
            InlineVectorBase<Elem, 4>::_vec[3] = d;
         }

         template<typename Elem2>
         explicit InlineVector(InlineVector<Elem2,4> const& a)
         {
            for(int i=0; i<4; i++)
               this->_vec[i] = (Elem2)a[i];
         }

       template <int size>
       InlineVector<Elem,size> slice( int first ) const
       {
           InlineVector<Elem,size> vec;
           copyVectorSlice(*this,first,size,vec,0);
           return vec;
       }
   }; // end struct InlineVector

   template <typename Elem>
   struct Vector : public VectorBase<Elem>
   {
         Vector()
            : VectorBase<Elem>()
         { }

         Vector(unsigned int size)
            : VectorBase<Elem>(size)
         { }

         Vector(unsigned int size, Elem * values)
            : VectorBase<Elem>(size, values)
         { }

         Vector(Vector<Elem> const& a)
            : VectorBase<Elem>(a)
         { }

         Vector<Elem>& operator=(Vector<Elem> const& a)
         {
            (VectorBase<Elem>::operator=)(a);
            return *this;
         }

         Vector<Elem>& operator+=(Vector<Elem> const& rhs)
         {
            addVectorsIP(rhs, *this);
            return *this;
         }

         Vector<Elem>& operator*=(Elem scale)
         {
            scaleVectorIP(scale, *this);
            return *this;
         }

         Vector<Elem> operator+(Vector<Elem> const& rhs) const
         {
            Vector<Elem> res(this->size());
            addVectors(*this, rhs, res);
            return res;
         }

         Vector<Elem> operator-(Vector<Elem> const& rhs) const
         {
            Vector<Elem> res(this->size());
            subtractVectors(*this, rhs, res);
            return res;
         }

         Elem operator*(Vector<Elem> const& rhs) const
         {
            return innerProduct(*this, rhs);
         }

   }; // end struct Vector

   template <typename Elem, int Rows, int Cols>
   struct InlineMatrix : public InlineMatrixBase<Elem, Rows, Cols>
   {
         template <int numRows, int numCols>
         InlineMatrix<Elem,numRows,numCols> slice( int row, int col ) const
         {
            InlineMatrix<Elem,numRows,numCols> mat;
            copyMatrixSlice(*this,row,col,numRows,numCols,mat,0,0);
            return mat;
         }

         InlineVector<Elem,Cols> row( int row ) const
         {
            InlineVector<Elem,Cols> vec;
            this->getRowSlice(row,0,Cols,vec);
            return vec;
         }

         InlineVector<Elem,Rows> col( int col ) const
         {
            InlineVector<Elem,Rows> vec;
            this->getColumnSlice(0,Rows,col,vec);
            return vec;
         }

       InlineMatrix<Elem,Cols,Rows> transposed() const
       {
           return transposedMatrix(*this);
       }
   }; // end struct InlineMatrix

   template <typename Elem>
   struct Matrix : public MatrixBase<Elem>
   {
         Matrix()
            : MatrixBase<Elem>()
         { }

         Matrix(unsigned int rows, unsigned int cols)
            : MatrixBase<Elem>(rows, cols)
         { }

         Matrix(unsigned int rows, unsigned int cols, Elem value)
            : MatrixBase<Elem>(rows, cols)
         {
            fillMatrix(*this, value);
         }

         Matrix(unsigned int rows, unsigned int cols, Elem * values)
            : MatrixBase<Elem>(rows, cols, values)
         { }

         Matrix(Matrix<Elem> const& a)
            : MatrixBase<Elem>(a)
         { }

         Matrix<Elem>& operator=(Matrix<Elem> const& a)
         {
            (MatrixBase<Elem>::operator=)(a);
            return *this;
         }

         Matrix<Elem>& operator+=(Matrix<Elem> const& rhs)
         {
            addMatricesIP(rhs, *this);
            return *this;
         }

         Matrix<Elem>& operator*=(Elem scale)
         {
            scaleMatrixIP(scale, *this);
            return *this;
         }

         Matrix<Elem> operator+(Matrix<Elem> const& rhs) const
         {
            Matrix<Elem> res(this->num_rows(), this->num_cols());
            addMatrices(*this, rhs, res);
            return res;
         }

         Matrix<Elem> operator-(Matrix<Elem> const& rhs) const
         {
            Matrix<Elem> res(this->num_rows(), this->num_cols());
            subtractMatrices(*this, rhs, res);
            return res;
         }

   }; // end struct Matrix

//----------------------------------------------------------------------

   typedef InlineVector<float, 2>  Vector2f;
   typedef InlineVector<double, 2> Vector2d;
   typedef InlineVector<float, 3>  Vector3f;
   typedef InlineVector<double, 3> Vector3d;
   typedef InlineVector<float, 4>  Vector4f;
   typedef InlineVector<double, 4> Vector4d;

   typedef InlineVector<unsigned char, 3>  Vector3b; // For color specifications e.g.

   typedef InlineMatrix<float, 2, 2>  Matrix2x2f;
   typedef InlineMatrix<double, 2, 2> Matrix2x2d;
   typedef InlineMatrix<float, 3, 3>  Matrix3x3f;
   typedef InlineMatrix<double, 3, 3> Matrix3x3d;
   typedef InlineMatrix<float, 4, 4>  Matrix4x4f;
   typedef InlineMatrix<double, 4, 4> Matrix4x4d;

   typedef InlineMatrix<float, 2, 3>  Matrix2x3f;
   typedef InlineMatrix<double, 2, 3> Matrix2x3d;
   typedef InlineMatrix<float, 3, 4>  Matrix3x4f;
   typedef InlineMatrix<double, 3, 4> Matrix3x4d;

   template <typename Elem>
   struct VectorArray
   {
         VectorArray(unsigned count, unsigned size)
            : _count(count), _size(size), _values(0), _vectors(0)
         {
            unsigned const nTotal = _count * _size;
            if (count > 0) _vectors = new Vector<Elem>[count];
            if (nTotal > 0) _values = new Elem[nTotal];
            for (unsigned i = 0; i < _count; ++i) new (&_vectors[i]) Vector<Elem>(_size, _values + i*_size);
         }

         VectorArray(unsigned count, unsigned size, Elem initVal)
            : _count(count), _size(size), _values(0), _vectors(0)
         {
            unsigned const nTotal = _count * _size;
            if (count > 0) _vectors = new Vector<Elem>[count];
            if (nTotal > 0) _values = new Elem[nTotal];
            for (unsigned i = 0; i < _count; ++i) new (&_vectors[i]) Vector<Elem>(_size, _values + i*_size);
            std::fill(_values, _values + nTotal, initVal);
         }

         ~VectorArray()
         {
            delete [] _values;
            delete [] _vectors;
         }

         unsigned count() const { return _count; }
         unsigned size()  const { return _size; }

         //! Get the submatrix at position ix
         Vector<Elem> const& operator[](unsigned ix) const
         {
            return _vectors[ix];
         }

         //! Get the submatrix at position ix
         Vector<Elem>& operator[](unsigned ix)
         {
            return _vectors[ix];
         }

      protected:
         unsigned       _count, _size;
         Elem         * _values;
         Vector<Elem> * _vectors;

      private:
         VectorArray(VectorArray const&);
         void operator=(VectorArray const&);
   };

   // Make a flat array look like a VectorArray, i.e. an array of vectors
   template <typename Elem>
   struct VectorArrayAdapter
   {
         VectorArrayAdapter(unsigned count, unsigned size, Elem * values)
            : _count(count), _size(size), _vectors(0)
         {
            if (count > 0) _vectors = new Vector<Elem>[count];
            for (unsigned i = 0; i < _count; ++i) new (&_vectors[i]) Vector<Elem>(_size, values + i*_size);
         }

         ~VectorArrayAdapter() { delete [] _vectors; }

         unsigned count() const { return _count; }
         unsigned size()  const { return _size; }

         //! Get the vector at position ix
         Vector<Elem> const& operator[](unsigned ix) const { return _vectors[ix]; }
         Vector<Elem>&       operator[](unsigned ix)       { return _vectors[ix]; }

      protected:
         unsigned       _count, _size;
         Vector<Elem> * _vectors;

      private:
         VectorArrayAdapter(VectorArrayAdapter const&);
         void operator=(VectorArrayAdapter const&);
   };

   template <typename Elem>
   struct MatrixArray
   {
         MatrixArray(unsigned count, unsigned nRows, unsigned nCols)
            : _count(count), _rows(nRows), _columns(nCols), _values(0), _matrices(0)
         {
            unsigned const nTotal = _count * _rows * _columns;
            if (count > 0) _matrices = new Matrix<Elem>[count];
            if (nTotal > 0) _values = new Elem[nTotal];
            for (unsigned i = 0; i < _count; ++i)
               new (&_matrices[i]) Matrix<Elem>(_rows, _columns, _values + i*(_rows*_columns));
         }

         ~MatrixArray()
         {
            delete [] _matrices;
            delete [] _values;
         }

         //! Get the submatrix at position ix
         Matrix<Elem> const& operator[](unsigned ix) const
         {
            return _matrices[ix];
         }

         //! Get the submatrix at position ix
         Matrix<Elem>& operator[](unsigned ix)
         {
            return _matrices[ix];
         }

         unsigned count()    const { return _count; }
         unsigned num_rows() const { return _rows; }
         unsigned num_cols() const { return _columns; }

      protected:
         unsigned       _count, _rows, _columns;
         Elem         * _values;
         Matrix<Elem> * _matrices;

      private:
         MatrixArray(MatrixArray const&);
         void operator=(MatrixArray const&);
   };

//----------------------------------------------------------------------

   template <typename Elem, int Size>
   inline InlineVector<Elem, Size+1>
   homogenizeVector(InlineVector<Elem, Size> const& v)
   {
      InlineVector<Elem, Size+1> res;
      copyVectorSlice(v, 0, Size, res, 0);
      res[Size] = 1;
      return res;
   }

   template <typename Elem, int Size>
   inline InlineVector<Elem, Size>
   operator-(InlineVector<Elem, Size> const& v)
   {
      InlineVector<Elem, Size> res;
      scaleVector(-1, v, res);
      return res;
   }

   template <typename Elem, int Size>
   inline InlineVector<Elem, Size>
   operator+(InlineVector<Elem, Size> const& v, InlineVector<Elem, Size> const& w)
   {
      InlineVector<Elem, Size> res;
      addVectors(v, w, res);
      return res;
   }

   template <typename Elem, int Size>
   inline InlineVector<Elem, Size>
   operator-(InlineVector<Elem, Size> const& v, InlineVector<Elem, Size> const& w)
   {
      InlineVector<Elem, Size> res;
      subtractVectors(v, w, res);
      return res;
   }

   template <typename Elem, int Rows, int Cols>
   inline InlineMatrix<Elem, Rows, Cols>
   operator-(InlineMatrix<Elem, Rows, Cols> const& A, InlineMatrix<Elem, Rows, Cols> const& B)
   {
      InlineMatrix<Elem, Rows, Cols> res;
      subtractMatrices(A, B, res);
      return res;
   }

   template <typename Elem, int Rows, int Cols>
   inline InlineMatrix<Elem, Rows, Cols>
   operator+(InlineMatrix<Elem, Rows, Cols> const& A, InlineMatrix<Elem, Rows, Cols> const& B)
   {
      InlineMatrix<Elem, Rows, Cols> res;
      addMatrices(A, B, res);
      return res;
   }

   template <typename Elem, int Size>
   inline InlineVector<Elem, Size>
   operator*(Elem scale, InlineVector<Elem, Size> const& v)
   {
      InlineVector<Elem, Size> res;
      scaleVector(scale, v, res);
      return res;
   }

   template <typename Elem, int Rows, int Cols>
   inline InlineMatrix<Elem, Rows, Cols>
   operator*(Elem scale, InlineMatrix<Elem, Rows, Cols> const& A)
   {
      InlineMatrix<Elem, Rows, Cols> res;
      scaleMatrix(A, scale, res);
      return res;
   }

   template <typename Elem, int Rows, int Cols>
   inline InlineVector<Elem, Rows>
   operator*(InlineMatrix<Elem, Rows, Cols> const& A, InlineVector<Elem, Cols> const& v)
   {
      InlineVector<Elem, Rows> res;
      multiply_A_v(A, v, res);
      return res;
   }

   template <typename Elem, int Rows, int Cols>
   inline InlineVector<Elem, Rows>
   operator*(InlineVector<Elem, Rows> const& v, InlineMatrix<Elem, Rows, Cols> const& A)
   {
      InlineVector<Elem, Cols> res;
      multiply_At_v(A, v, res);
      return res;
   }

   template <typename Elem, int RowsA, int ColsA, int ColsB>
   inline InlineMatrix<Elem, RowsA, ColsB>
   operator*(InlineMatrix<Elem, RowsA, ColsA> const& A, InlineMatrix<Elem, ColsA, ColsB> const& B)
   {
      InlineMatrix<Elem, RowsA, ColsB> res;
      multiply_A_B(A, B, res);
      return res;
   }

   template <typename Elem, int Rows, int Cols>
   inline InlineMatrix<Elem, Cols, Rows>
   transposedMatrix(InlineMatrix<Elem, Rows, Cols> const& A)
   {
      InlineMatrix<Elem, Cols, Rows> At;
      makeTransposedMatrix(A, At);
      return At;
   }

   template <typename Elem>
   inline InlineMatrix<Elem, 2, 2>
   invertedMatrix(InlineMatrix<Elem, 2, 2> const& A)
   {
      Elem a = A[0][0], b = A[0][1], c = A[1][0], d = A[1][1];

      Elem const det = a*d - b*c;

      InlineMatrix<Elem, 2, 2> res;
      res[0][0] = d;  res[0][1] = -b;
      res[1][0] = -c; res[1][1] = a;

      scaleMatrixIP(1.0/det, res);
      return res;
   }

   template <typename Elem>
   inline InlineMatrix<Elem, 3, 3>
   invertedMatrix(InlineMatrix<Elem, 3, 3> const& A)
   {
      Elem a = A[0][0], b = A[0][1], c = A[0][2];
      Elem d = A[1][0], e = A[1][1], f = A[1][2];
      Elem g = A[2][0], h = A[2][1], i = A[2][2];

      Elem const det = a*e*i + b*f*g + c*d*h - c*e*g - b*d*i - a*f*h;

      InlineMatrix<Elem, 3, 3> res;
      res[0][0] = e*i-f*h; res[0][1] = c*h-b*i; res[0][2] = b*f-c*e;
      res[1][0] = f*g-d*i; res[1][1] = a*i-c*g; res[1][2] = c*d-a*f;
      res[2][0] = d*h-e*g; res[2][1] = b*g-a*h; res[2][2] = a*e-b*d;

      scaleMatrixIP(1.0/det, res);
      return res;
   }

   template <typename Elem, int Rows, int Cols>
   inline InlineMatrix<Elem,Rows,Cols>
   outerProductMatrix(InlineVector<Elem,Rows> const& u, InlineVector<Elem,Cols> const& v)
   {
      InlineMatrix<Elem,Rows,Cols> mat;
      makeOuterProductMatrix(u,v,mat);
      return mat;
   }

   template <typename Elem>
   inline InlineMatrix<Elem, 3, 3>
   crossProductMatrix(InlineVector<Elem, 3> const& v)
   {
      InlineMatrix<Elem, 3, 3> res;
      makeCrossProductMatrix(v, res);
      return res;
   }

   template <typename Elem>
   inline InlineVector<Elem,3>
   crossProduct(InlineVector<Elem,3> const& u, InlineVector<Elem,3> const& v)
   {
      InlineVector<Elem,3> res;
      makeCrossProductVector(u,v,res);
      return res;
   }

   template <typename Elem>
   inline InlineVector<Elem, 2>
   makeVector2(Elem a, Elem b)
   {
      InlineVector<Elem, 2> res;
      res[0] = a; res[1] = b;
      return res;
   }

   template <typename Elem>
   inline InlineVector<Elem, 3>
   makeVector3(Elem a, Elem b, Elem c)
   {
      InlineVector<Elem, 3> res;
      res[0] = a; res[1] = b; res[2] = c;
      return res;
   }

   template <typename Elem>
   inline InlineVector<Elem, 4>
   makeVector4(Elem a, Elem b, Elem c, Elem d)
   {
      InlineVector<Elem, 4> res;
      res[0] = a; res[1] = b; res[2] = c; res[3] = d;
      return res;
   }

//======================================================================

   template <typename Elem>
   inline Vector<Elem>
   operator*(Matrix<Elem> const& A, Vector<Elem> const& v)
   {
      Vector<Elem> res(A.num_rows());
      multiply_A_v(A, v, res);
      return res;
   }

//======================================================================

   template <typename Vec>
   inline void
   displayVector(Vec const& v)
   {
      using namespace std;

      cout << "[ ";
      for (int r = 0; r < v.size(); ++r)
         cout << v[r] << " ";
      cout << "]" << endl;
   }

   template <typename Mat>
   inline void
   displayMatrix(Mat const& A)
   {
      using namespace std;

      cout << "[ ";
      for (int r = 0; r < A.num_rows(); ++r)
      {
         for (int c = 0; c < A.num_cols(); ++c)
            cout << A[r][c] << " ";
         if (r < A.num_rows()-1)
            cout << endl;
         else
            cout << "]" << endl;
      }
   }

   template <typename SparseMat>
   inline void
   displaySparseMatrix(SparseMat const& A)
   {
      int const n = A.num_rows();
      int const m = A.num_cols();

      typedef typename SparseMat::value_type Elem;

      Matrix<Elem> denseA(n, m);
      makeZeroMatrix(denseA);

      std::vector<int>  rows(n);
      std::vector<Elem> vals(n);

      for (int j = 0; j < m; ++j)
      {
         int const nnz = A.getColumnNonzeroCount(j);
         A.getSparseColumn(j, rows, vals);

         for (int k = 0; k < nnz; ++k)
         {
            int const i = rows[k];
            denseA[i][j] = vals[k];
         }
      } // end for (j)
      displayMatrix(denseA);
   } // end displaySparseMatrix()

   template <typename Elem>
   inline void
   showSparseMatrixInfo(CCS_Matrix<Elem> const& A)
   {
      int const nCols = A.num_cols();
      int const nnz = A.getNonzeroCount();
      int const * colStarts = A.getColumnStarts();
      int const * rowIdxs   = A.getRowIndices();
      int const * destIdxs  = A.getDestIndices();
      Elem const * values = A.getValues();

      cout << "colStarts = ";
      for (int k = 0; k <= nCols; ++k) cout << colStarts[k] << " ";
      cout << endl;

      cout << "rowIdxs = ";
      for (int k = 0; k < nnz; ++k) cout << rowIdxs[k] << " ";
      cout << endl;

      cout << "destIdxs = ";
      for (int k = 0; k < nnz; ++k) cout << destIdxs[k] << " ";
      cout << endl;

      cout << "values = ";
      for (int k = 0; k < nnz; ++k) cout << values[k] << " ";
      cout << endl;
   } // end showSparseMatrixInfo()

} // end namespace V3D

#endif
