// -*- C++ -*-

#ifndef V3D_SERIALIZATION_H
#define V3D_SERIALIZATION_H

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <cstring>

#include "Base/v3d_exception.h"

//! Adds load and save routines to the serializable struct.
#define V3D_DEFINE_LOAD_SAVE(T)                                         \
   template <typename Ar>                                               \
   void save(Ar& ar) const { T& self = const_cast<T&>(*this); self.serialize(ar); } \
   template <typename Ar>                                               \
   void load(Ar& ar)       { this->serialize(ar); }                     \
   template <typename Ar>                                               \
   void serialize(Ar& ar,const unsigned int& file_version) const {      \
      T& self = const_cast<T&>(*this); self.serialize(ar);              \
   }

//! Implements \c << and \c >> operators for streams for serializable structs.
#define V3D_DEFINE_IOSTREAM_OPS(T)                                      \
   inline std::istream& operator>>(std::istream& is, T& v)              \
   {                                                                    \
      return V3D::loadFromIStream(is, v);                 \
   }                                                                    \
   inline std::ostream& operator<<(std::ostream& os, T const& v)        \
   {                                                                    \
      return V3D::saveToOStream(os, v);                   \
   }

//! Implements \c << and \c >> operators for streams for templated serializable structs.
#define V3D_DEFINE_TEMPLATE_IOSTREAM_OPS(T)                             \
   template <typename TT>                                               \
   inline std::istream& operator>>(std::istream& is, T<TT>& v)          \
   {                                                                    \
      return V3D::loadFromIStream(is, v);                 \
   }                                                                    \
   template <typename TT>                                               \
   inline std::ostream& operator<<(std::ostream& os, T<TT> const& v)    \
   {                                                                    \
      return V3D::saveToOStream(os, v);                   \
   }

namespace V3D
{

   template <typename Archive>
   struct OArchiveProtocol
   {
         Archive * archive()
         {
            return static_cast<Archive *>(this);
         }

         static bool isLoading() { return false; }
         static bool isSaving()  { return true; }

         template <typename T>
         Archive& operator<<(T const& val)
         {
            this->archive()->save(val);
            return *this->archive();
         }

         template <typename T>
         Archive& operator&(T const& val)
         {
            return (*this) << val;
         }

         void serializeBlob(void * address, size_t count)
         {
            this->archive()->serializeBlob(address, count);
         }

         //! Do not use whitespace in \param tag!
         void tag(char const * tag)
         {
            this->archive()->saveTag(tag);
         }

         void enterScope() { this->archive()->saveTag("{"); }
         void leaveScope() { this->archive()->saveTag("}"); }
         void endl()       { this->archive()->endl(); }
   };

   template <typename Archive>
   struct IArchiveProtocol
   {
         Archive * archive()
         {
            return static_cast<Archive *>(this);
         }

         static bool isLoading() { return true; }
         static bool isSaving()  { return false; }

         template <typename T>
         Archive& operator>>(T& val)
         {
            this->archive()->load(val);
            return *this->archive();
         }

         template <typename T>
         Archive& operator&(T& val)
         {
            return (*this) >> val;
         }

         void serializeBlob(void * address, size_t count)
         {
            this->archive()->serializeBlob(address, count);
         }

         //! Do not use whitespace in \param tag!
         void tag(char const * tag)
         {
            std::string stag(tag);
            std::string s;
            this->archive()->loadTag(s);
            if (stag != s)
               throwV3DErrorHere(std::string("Tag mismatch <" + s + "> instead of <") + stag + std::string(">"));
         }

         void enterScope()
         {
            std::string s;
            this->archive()->loadTag(s);
            if (s != "{") throwV3DErrorHere("Bracket mismatch <{>");
         }

         void leaveScope()
         {
            std::string s;
            this->archive()->loadTag(s);
            if (s != "}") throwV3DErrorHere("Bracket mismatch <}>");
         }

         void endl() { }
   };

   template <typename Archive>
   struct SerializationScope
   {
         SerializationScope(Archive& ar)
            : _ar(ar)
         {
            ar.enterScope();
         }

         SerializationScope(Archive& ar, char const * tag)
            : _ar(ar)
         {
            ar.tag(tag);
            ar.enterScope();
         }

         ~SerializationScope()
         {
            _ar.leaveScope();
         }

      protected:
         Archive& _ar;
   }; // end struct SerializationScope

//----------------------------------------------------------------------

   struct TextOStreamArchive : public OArchiveProtocol<TextOStreamArchive>
   {
      private:
         typedef OArchiveProtocol<TextOStreamArchive> Base;

      public:
         TextOStreamArchive(std::ostream& os)
            : Base(), _os(os),
              _indentation(0)
         { }

         template <typename T> void save(T const& v) { v.T::save(*this); }

         void save(bool val)               { this->put((int)val); }
         void save(unsigned int val)       { this->put(val); }
         void save(int val)                { this->put(val); }
         void save(long val)               { this->put(val); }
         void save(unsigned long val)      { this->put(val); }
         void save(short int val)          { this->put(val); }
         void save(unsigned short int val) { this->put(val); }
         void save(float val)              { this->put(val); }
         void save(double val)             { this->put(val); }
         void save(char val)               { this->put(int(val)); }
         void save(unsigned char val)      { this->put((unsigned int)val); }

         void save(char const * str)
         {
            unsigned int len = std::strlen(str);
            this->save(len);
            _os.write(str, len);
            _os << " ";
         }

         void save(std::string const& str)
         {
            unsigned int len = str.length();
            this->save(len);
            _os.write(str.c_str(), len);
            _os << " ";
         }

         template <typename T1,typename T2>
            void save(std::pair<T1,T2> const& v) {
               save(const_cast<T1&>(v.first));
               save(const_cast<T2&>(v.second));
            }

         void serializeBlob(void * address, size_t count)
         {
            char const * buffer = static_cast<const char *>(address);
            _os.write(buffer, count);
         }

         void saveTag(char const * tag) { this->put(tag); }

         void enterScope()
         {
            _indentation += 2;
            Base::enterScope();
         }

         void leaveScope()
         {
            _indentation -= 2;
            Base::leaveScope();
            this->endl();
         }

         void endl()
         {
            _os << std::endl;
            this->indent();
         }

      protected:
         template <typename T>
         void put(T const& v)
         {
            _os << v << " ";
         }

         void indent()
         {
            for (int i = 0; i < _indentation; ++i) _os << " ";
         }

         std::ostream& _os;
         int           _indentation;
   };

   struct TextIStreamArchive : public IArchiveProtocol<TextIStreamArchive>
   {
      private:
         typedef IArchiveProtocol<TextIStreamArchive> Base;

      public:
         TextIStreamArchive(std::istream& is)
            : Base(), _is(is)
         { }

         template <typename T> void load(T& v) { v.T::load(*this); }

         void load(bool& val)               { int v; get(v); val = (v != 0) ? true : false; }
         void load(unsigned int& val)       { get(val); }
         void load(int& val)                { get(val); }
         void load(long& val)               { get(val); }
         void load(unsigned long& val)      { get(val); }
         void load(short int& val)          { get(val); }
         void load(unsigned short int& val) { get(val); }
         void load(float& val)              { get(val); }
         void load(double& val)             { get(val); }
         void load(char& val)               { int v; get(v); val = v; }
         void load(unsigned char& val)      { unsigned v; get(v); val = v; }

         void load(char * str)
         {
            unsigned int len;
            this->load(len);
            _is.ignore(); // Ignore the extra blank after len
            _is.read(str, len);
            str[len] = 0;
         }

         void load(std::string& str)
         {
            unsigned int len;
            this->load(len);
            _is.ignore(); // Ignore the extra blank after len
            std::vector<char> buf(len+1);
            _is.read(&buf[0], len);
            buf[len] = 0;
            str = &buf[0];
         }

         template <typename T1,typename T2>
            void load(std::pair<T1,T2> & v) {
               load(v.first);
               load(v.second);
            }

         void serializeBlob(void * address, size_t count)
         {
            char * buffer = static_cast<char *>(address);
            _is.read(buffer, count);
         }

         void loadTag(std::string& tag)
         {
            _is >> std::ws >> tag;
         }

         void enterScope() { Base::enterScope(); }
         void leaveScope() { Base::leaveScope(); }

      protected:
         template <typename T>
         void get(T& v)
         {
            _is >> v;
         }

         std::istream& _is;
   };

//----------------------------------------------------------------------

   //! Output archive using a binary stream to write.
   /*! Integer values (short, int, long) are stored as 32 bit entities.
    * Hence binary serialization is not dependent on sizeof(int) etc.
    * Floating point types are directly written, but all relevant platforms use
    * IEEE fp format anyway.
    * Chars and bools are written as 8 bit entities.
    * Do not forget the ios::binary attribute for stream opening!
    */
   struct BinaryOStreamArchive : public OArchiveProtocol<BinaryOStreamArchive>
   {
      private:
         typedef OArchiveProtocol<BinaryOStreamArchive> Base;

      public:
         BinaryOStreamArchive(std::ostream& os)
            : Base(),_os(os)
         { }

         template <typename T> void save(T const& v) { v.save(*this); }

         void save(bool val)               { this->put_byte(val); }
         void save(unsigned int val)       { this->put_uint32(val); }
         void save(int val)                { this->put_int32(val); }
         void save(long val)               { this->put_int32(val); }
         void save(unsigned long val)      { this->put_uint32(val); }
         void save(short int val)          { this->put_int32(val); }
         void save(unsigned short int val) { this->put_uint32(val); }
         void save(float val)              { this->put_blob(val); }
         void save(double val)             { this->put_blob(val); }
         void save(char val)               { this->put_byte(val); }
         void save(unsigned char val)      { this->put_byte(val); }

         void save(char const * str)
         {
            //assert(static_cast<unsigned int>(std::strlen(str)) < std::numeric_limits<unsigned int>::max());
            unsigned int len = static_cast<unsigned int>(std::strlen(str));
            save(len);
            _os.write(str, len);
         }

         void save(std::string const& str)
         {
            //assert(static_cast<unsigned int>(str.length()) < std::numeric_limits<unsigned int>::max());
            unsigned int len = static_cast<unsigned int>(str.length());
            save(len);
            _os.write(str.c_str(), len);
         }
         template <typename T1,typename T2>
            void save(std::pair<T1,T2> const& v) {
               save(const_cast<T1&>(v.first));
               save(const_cast<T2&>(v.second));
            }

         void serializeBlob(void * address, size_t count)
         {
            char const * buffer = static_cast<const char *>(address);
            _os.write(buffer, count);
         }

         void saveTag(char const *) { }
         void enterScope() { }
         void leaveScope() { }
         void endl() { }

      protected:
         template <typename T>
         void put_byte(T v)
         {
            char val = v;
            _os.write(&val, 1);
         }

         void put_int32(signed long v)
         {
            unsigned char buf[4];
            buf[0] = static_cast<unsigned char>(v & 0xff);
            buf[1] = static_cast<unsigned char>((v >> 8) & 0xff);
            buf[2] = static_cast<unsigned char>((v >> 16) & 0xff);
            buf[3] = static_cast<unsigned char>((v >> 24) & 0xff);
            _os.write((char *)buf, 4);
         }

         void put_uint32(unsigned long v)
         {
            unsigned char buf[4];
            buf[0] = static_cast<unsigned char>(v & 0xff);
            buf[1] = static_cast<unsigned char>((v >> 8) & 0xff);
            buf[2] = static_cast<unsigned char>((v >> 16) & 0xff);
            buf[3] = static_cast<unsigned char>((v >> 24) & 0xff);
            _os.write((char *)buf, 4);
         }

         template <typename T>
         void put_blob(T v)
         {
            _os.write((char *)&v, sizeof(v));
         }

         std::ostream& _os;
   };

   struct BinaryIStreamArchive : public IArchiveProtocol<BinaryIStreamArchive>
   {
      private:
         typedef IArchiveProtocol<BinaryIStreamArchive> Base;

      public:
         BinaryIStreamArchive(std::istream& is)
            : Base(), _is(is)
         { }

         template <typename T> void load(T& v) { v.load(*this); }

         void load(bool& val)               { get_uchar(val); }
         void load(unsigned int& val)       { getUnsigned(val); }
         void load(int& val)                { getSigned(val); }
         void load(long& val)               { getSigned(val); }
         void load(unsigned long& val)      { getUnsigned(val); }
         void load(short int& val)          { getSigned(val); }
         void load(unsigned short int& val) { getUnsigned(val); }
         void load(float& val)              { getGeneric(val); }
         void load(double& val)             { getGeneric(val); }
         void load(char& val)               { get_schar(val); }
         void load(unsigned char& val)      { get_uchar(val); }

         void load(char * str)
         {
            unsigned int len;
            load(len);
            _is.read(str, len);
            str[len] = 0;
         }

         void load(std::string& str)
         {
            unsigned int len;
            this->load(len);
            std::vector<char> buf(len+1);
            _is.read(&buf[0], len);
            buf[len] = 0;
            str = &buf[0];
         }

         template <typename T1,typename T2>
            void load(std::pair<T1,T2> & v) {
               load(v.first);
               load(v.second);
            }

         void serializeBlob(void * address, size_t count)
         {
            char * buffer = static_cast<char *>(address);
            _is.read(buffer, count);
         }

         void loadTag(std::string& /*tag*/)     { }
         void tag(std::string const& /*tag*/)   { }
         void enterScope() { }
         void leaveScope() { }

         void skip(unsigned int nBytes) { _is.ignore(nBytes); }

      protected:
         template <typename T>
         void get_uchar(T& v)
         {
            unsigned char val;
            _is.read((char *)&val, 1);
            v = val;
         }

         template <typename T>
         void get_schar(T& v)
         {
            char val;
            _is.read((char *)&val, 1);
            v = val;
         }

         template <typename T>
         void getSigned(T& v)
         {
            signed long val;
            get_int32(val);
            v = val;
         }

         template <typename T>
         void getUnsigned(T& v)
         {
            unsigned long val;
            get_uint32(val);
            v = val;
         }

         void get_uint32(unsigned long& v)
         {
            unsigned char buf[4];
            _is.read((char *)buf, 4);
            v = buf[0] + (buf[1] << 8) + (buf[2] << 16) + (buf[3] << 24);
         }

         void get_int32(signed long& v)
         {
            unsigned char buf[4];
            _is.read((char *)buf, 4);
            v = buf[0] + (buf[1] << 8) + (buf[2] << 16);
            // The following is somewhat magic, interpret the most significant byte as signed char.
            v += (signed char)(buf[3]) << 24;
         }

         template <typename T>
         void getGeneric(T& v)
         {
            _is.read((char *)&v, sizeof(v));
         }

         std::istream& _is;
   };

//----------------------------------------------------------------------

   struct BinaryArchiveSizeAccumulator : public OArchiveProtocol<BinaryArchiveSizeAccumulator>
   {
         BinaryArchiveSizeAccumulator()
            : _byteSize(0)
         { }

         template <typename T> void save(T const& v) { v.save(*this); }

         void save(bool)               { _byteSize += 1; }
         void save(unsigned int)       { _byteSize += 4; }
         void save(int)                { _byteSize += 4; }
         void save(long)               { _byteSize += 4; }
         void save(unsigned long)      { _byteSize += 4; }
         void save(short int)          { _byteSize += 4; }
         void save(unsigned short int) { _byteSize += 4; }
         void save(float)              { _byteSize += 4; }
         void save(double)             { _byteSize += 8; }
         void save(char)               { _byteSize += 1; }
         void save(unsigned char)      { _byteSize += 1; }

         void save(char const * str)
         {
            unsigned int len = std::strlen(str);
            this->save(len);
            _byteSize += len;
         }

         void save(std::string const& str)
         {
            unsigned int len = str.length();
            this->save(len);
            _byteSize += len;
         }

         template <typename T1,typename T2>
            void save(std::pair<T1,T2> const& v) {
               save(const_cast<T1&>(v.first));
               save(const_cast<T2&>(v.second));
            }

         void serializeBlob(void *, size_t count)
         {
            _byteSize += count;
         }

         void tag(std::string const&) { }
         void enterScope() { }
         void leaveScope() { }
         void endl()       { }

         unsigned int byteSize() const { return _byteSize; }

      protected:
         unsigned int _byteSize;
   }; // end struct BinaryArchiveSizeAccumulator

//----------------------------------------------------------------------

   // Serialize to a blob (contiguous memory)
   struct BlobOArchive : public OArchiveProtocol<BlobOArchive>
   {
      private:
         typedef OArchiveProtocol<BlobOArchive> Base;

      public:
         BlobOArchive(int sz = 0)
            : Base()
         {
            if (sz > 0) _blob.reserve(sz);
         }

         void clear() { _blob.clear(); }

         unsigned char const * getBlob() const { return &_blob[0]; }
         int                   blobSize() const { return _blob.size(); }

         template <typename T> void save(T const& v) { v.save(*this); }

         void save(bool val)               { this->put_byte(val); }
         void save(unsigned int val)       { this->put_uint32(val); }
         void save(int val)                { this->put_int32(val); }
         void save(long val)               { this->put_int32(val); }
         void save(unsigned long val)      { this->put_uint32(val); }
         void save(short int val)          { this->put_int32(val); }
         void save(unsigned short int val) { this->put_uint32(val); }
         void save(float val)              { this->put_blob(val); }
         void save(double val)             { this->put_blob(val); }
         void save(char val)               { this->put_byte(val); }
         void save(unsigned char val)      { this->put_byte(val); }

         void save(char const * str)
         {
            unsigned int len = static_cast<unsigned int>(std::strlen(str));
            this->save(len);
            this->serializeBlob(str, len);
         }

         void save(std::string const& str)
         {
            unsigned int len = static_cast<unsigned int>(str.length());
            this->save(len);
            this->serializeBlob(str.c_str(), len);
         }

         template <typename T1,typename T2>
            void save(std::pair<T1,T2> const& v) {
               save(const_cast<T1&>(v.first));
               save(const_cast<T2&>(v.second));
            }

         void serializeBlob(void const * address, size_t count)
         {
            unsigned char const * buffer = static_cast<unsigned char const *>(address);
            for (size_t i = 0; i < count; ++i)
               _blob.push_back(buffer[i]);
         }

         void saveTag(char const *) { }
         void enterScope() { }
         void leaveScope() { }
         void endl() { }

      protected:
         template <typename T>
         void put_byte(T v)
         {
            unsigned char val = v;
            _blob.push_back(val);
         }

         void put_int32(signed long v)
         {
            unsigned char buf[4];
            buf[0] = static_cast<unsigned char>(v & 0xff);
            buf[1] = static_cast<unsigned char>((v >> 8) & 0xff);
            buf[2] = static_cast<unsigned char>((v >> 16) & 0xff);
            buf[3] = static_cast<unsigned char>((v >> 24) & 0xff);
            _blob.push_back(buf[0]);
            _blob.push_back(buf[1]);
            _blob.push_back(buf[2]);
            _blob.push_back(buf[3]);
         }

         void put_uint32(unsigned long v)
         {
            unsigned char buf[4];
            buf[0] = static_cast<unsigned char>(v & 0xff);
            buf[1] = static_cast<unsigned char>((v >> 8) & 0xff);
            buf[2] = static_cast<unsigned char>((v >> 16) & 0xff);
            buf[3] = static_cast<unsigned char>((v >> 24) & 0xff);
            _blob.push_back(buf[0]);
            _blob.push_back(buf[1]);
            _blob.push_back(buf[2]);
            _blob.push_back(buf[3]);
         }

         template <typename T>
         void put_blob(T v)
         {
            this->serializeBlob(&v, sizeof(v));
         }

         std::vector<unsigned char> _blob;
   };

   struct BlobIArchive : public IArchiveProtocol<BlobIArchive>
   {
      private:
         typedef IArchiveProtocol<BlobIArchive> Base;

      public:
         BlobIArchive(unsigned char const * blobStart)
            : Base(), _blobPtr(blobStart)
         { }

         template <typename T> void load(T& v) { v.load(*this); }

         void load(bool& val)               { get_uchar(val); }
         void load(unsigned int& val)       { getUnsigned(val); }
         void load(int& val)                { getSigned(val); }
         void load(long& val)               { getSigned(val); }
         void load(unsigned long& val)      { getUnsigned(val); }
         void load(short int& val)          { getSigned(val); }
         void load(unsigned short int& val) { getUnsigned(val); }
         void load(float& val)              { getGeneric(val); }
         void load(double& val)             { getGeneric(val); }
         void load(char& val)               { get_schar(val); }
         void load(unsigned char& val)      { get_uchar(val); }

         void load(char * str)
         {
            unsigned int len;
            this->load(len);
            this->serializeBlob(str, len);
            str[len] = 0;
         }

         void load(std::string& str)
         {
            unsigned int len;
            this->load(len);
            std::vector<char> buf(len+1);
            this->serializeBlob(&buf[0], len);
            buf[len] = 0;
            str = &buf[0];
         }

         template <typename T1,typename T2>
            void load(std::pair<T1,T2> & v) {
               load(v.first);
               load(v.second);
            }

         void serializeBlob(void * address, size_t count)
         {
            unsigned char * buffer = static_cast<unsigned char *>(address);
            for (size_t i = 0; i < count; ++i)
               buffer[i] = _blobPtr[i];
            _blobPtr += count;
         }

         void loadTag(std::string& /*tag*/)     { }
         void tag(std::string const& /*tag*/)   { }
         void enterScope() { }
         void leaveScope() { }

         void skip(unsigned int nBytes) { _blobPtr += nBytes; }

      protected:
         template <typename T>
         void get_uchar(T& v)
         {
            v = *_blobPtr++;
         }

         template <typename T>
         void get_schar(T& v)
         {
            v = *_blobPtr++;
         }

         template <typename T>
         void getSigned(T& v)
         {
            signed long val;
            get_int32(val);
            v = val;
         }

         template <typename T>
         void getUnsigned(T& v)
         {
            unsigned long val;
            get_uint32(val);
            v = val;
         }

         void get_uint32(unsigned long& v)
         {
            unsigned char const * buf = _blobPtr;
            v = buf[0] + (buf[1] << 8) + (buf[2] << 16) + (buf[3] << 24);
            _blobPtr += 4;
         }

         void get_int32(signed long& v)
         {
            unsigned char const * buf = _blobPtr;
            v = buf[0] + (buf[1] << 8) + (buf[2] << 16);
            // The following is somewhat magic, interpret the most significant byte as signed char.
            v += (signed char)(buf[3]) << 24;
            _blobPtr += 4;
         }

         template <typename T>
         void getGeneric(T& v)
         {
            this->serializeBlob(&v, sizeof(v));
         }

         unsigned char const * _blobPtr;
   };

//----------------------------------------------------------------------

   //! Serializes a vector of serializable items.
   template <typename Archive, typename T>
   inline void
   serializeVector(std::vector<T>& v, Archive& ar)
   {
      unsigned int sz = v.size();
      ar & sz;
      if (ar.isLoading()) v.resize(sz);
      SerializationScope<Archive> s(ar);
      for (unsigned i = 0; i < sz; ++i) ar & v[i];
   }

   //! Serializes a vector of bool (needed as it is NOT a normal vector)
   template <typename Archive>
   inline void
   serializeVector(std::vector<bool>& v, Archive& ar)
   {
      unsigned int sz = v.size();
      ar & sz;
      if (ar.isLoading()) v.resize(sz);
      SerializationScope<Archive> s(ar);
      const unsigned int packsize=32;
      unsigned int realsz=(sz-1)/packsize+1;
      for (unsigned i = 0; i <realsz; ++i)
      {
         const unsigned int offset=i*packsize;
         unsigned int buf=0;
         if (ar.isSaving())
            for (unsigned j = 0; j <packsize; ++j)
               if(v[offset+j])buf|=(1<<j);
         ar & buf;
         if (ar.isLoading())
            for (unsigned j = 0; j <packsize; ++j)
               v[offset+j]=((buf>>j)&1)!=0;
      }
   }

   template <typename Archive, typename T>
   inline void
   serializeVector(char const * tag, std::vector<T>& v, Archive& ar)
   {
      ar.tag(tag);
      serializeVector(v, ar);
   }

   template <typename Archive, typename T>
   inline void
   serializeSet(std::set<T>& v, Archive& ar)
   {
      unsigned int sz = v.size();
      ar & sz;

      SerializationScope<Archive> s(ar);

      if (ar.isLoading())
      {
         T elem;
         for (unsigned i = 0; i < sz; ++i)
         {
            ar & elem;
            v.insert(elem);
         }
      }
      else
      {
         T elem;
         for (typename std::set<T>::iterator p = v.begin(); p != v.end(); ++p)
         {
            elem = *p;
            ar & elem;
         }
      } // end if
   } // end serializeSet()

   template <typename Archive, typename Key, typename T>
   inline void
   serializeMap(std::map<Key, T>& v, Archive& ar)
   {
      unsigned int sz = v.size();
      ar & sz;

      SerializationScope<Archive> s(ar);

      if (ar.isLoading())
      {
         v.clear();
         Key key;
         T elem;
         for (unsigned i = 0; i < sz; ++i)
         {
            ar & key & elem;
            v.insert(make_pair(key, elem));
         }
      }
      else
      {
         Key key;
         for (typename std::map<Key, T>::iterator p = v.begin(); p != v.end(); ++p)
         {
            key = p->first;
            ar & key & p->second;
         }
      } // end if
   } // end serializeMap()

   template <typename T>
   inline void
   serializeDataToFile(char const * archiveName, T const& data, bool writeBinary = true)
   {
      using namespace std;

      int const tagLength = 6;

      if (writeBinary)
      {
         char const * magicTag = "V3DBIN";
         ofstream os(archiveName, ios::binary);
         os.write(magicTag, tagLength);
         BinaryOStreamArchive ar(os);
         ar & data;
      }
      else
      {
         char const * magicTag = "V3DTXT";
         ofstream os(archiveName);
         os << magicTag << endl;
         TextOStreamArchive ar(os);
         ar & data;
      }
   } // end serializeDataToFile()

   template <typename T>
   inline void
   serializeDataFromFile(char const * archiveName, T& data)
   {
      using namespace std;

      int const tagLength = 6;

      bool isBinary = true;

      {
         // Determine archive format from the first 8 chars
         char magicTag[tagLength];
         ifstream is(archiveName, ios::binary);
         is.read(magicTag, tagLength);
         if (strncmp(magicTag, "V3DBIN", tagLength) == 0)
            isBinary = true;
         else if (strncmp(magicTag, "V3DTXT", tagLength) == 0)
            isBinary = false;
         else
            throwV3DErrorHere("Unknown archive magic tag");
      }

      if (isBinary)
      {
         ifstream is(archiveName, ios::binary);
         is.ignore(tagLength);
         BinaryIStreamArchive ar(is);
         ar & data;
      }
      else
      {
         ifstream is(archiveName);
         is.ignore(tagLength);
         TextIStreamArchive ar(is);
         ar & data;
      }
   } // end serializeDataFromFile()

   template <typename T>
   inline std::ostream&
   saveToOStream(std::ostream& os, T const& v)
   {
      using namespace std;
      TextOStreamArchive ar(os);
      ar << v;
      return os;
   }

   template <typename T>
   inline std::istream&
   loadFromIStream(std::istream& is, T& v)
   {
      using namespace std;
      TextIStreamArchive ar(is);
      ar >> v;
      return is;
   }

//----------------------------------------------------------------------

   template <typename Feature>
   struct SerializableVector : public std::vector<Feature>
   {
         SerializableVector()
            : std::vector<Feature>()
         { }

         SerializableVector(size_t sz)
            : std::vector<Feature>(sz)
         { }
         SerializableVector(std::vector<Feature> const & v)
            : std::vector<Feature>(v)
         { }
         template <typename Archive> void serialize(Archive& ar)
         {
            serializeVector(*this, ar);
         }
         V3D_DEFINE_LOAD_SAVE(SerializableVector)
   }; // end struct SerializableVector
   V3D_DEFINE_TEMPLATE_IOSTREAM_OPS(SerializableVector)

   template <typename Feature>
   struct SerializableSet : public std::set<Feature>
   {
         SerializableSet()
            : std::set<Feature>()
         { }

         SerializableSet(size_t sz)
            : std::set<Feature>(sz)
         { }
         SerializableSet(std::set<Feature> const & v)
            : std::set<Feature>(v)
         { }

         template <typename Archive> void serialize(Archive& ar)
         {
            serializeSet(*this, ar);
         }

         V3D_DEFINE_LOAD_SAVE(SerializableSet)
   }; // end struct SerializableSet
   V3D_DEFINE_TEMPLATE_IOSTREAM_OPS(SerializableSet)

   template <typename Key,typename Feature>
   struct SerializableMap : public std::map<Key,Feature>
   {
         SerializableMap()
            : std::map<Key,Feature>()
         { }

         SerializableMap(size_t sz)
            : std::map<Key,Feature>(sz)
         { }
         SerializableMap(std::map<Key,Feature> const & v)
            : std::map<Key,Feature>(v)
         { }
         template <typename Archive> void serialize(Archive& ar)
         {
            serializeMap(*this, ar);
         }

         V3D_DEFINE_LOAD_SAVE(SerializableMap)
   }; // end struct SerializableSet
   //V3D_DEFINE_TEMPLATE_IOSTREAM_OPS(SerializableMap) //Does not work with 2 template names

} // end namespace V3D

#endif
