#pragma once
#include "gl-util/gl-common.h"

namespace GL {

// Maps C++ type to GLenum type:
//  float -> GL_FLOAT
//  double -> GL_DOUBLE
//  etc.
template<typename T>
struct TypeInfo {};

template<> struct TypeInfo<float         > { static constexpr GLenum gl_type = GL_FLOAT         ; };
template<> struct TypeInfo<double        > { static constexpr GLenum gl_type = GL_DOUBLE        ; };
template<> struct TypeInfo<signed char   > { static constexpr GLenum gl_type = GL_BYTE          ; };
template<> struct TypeInfo<unsigned char > { static constexpr GLenum gl_type = GL_UNSIGNED_BYTE ; };
template<> struct TypeInfo<short         > { static constexpr GLenum gl_type = GL_SHORT         ; };
template<> struct TypeInfo<unsigned short> { static constexpr GLenum gl_type = GL_UNSIGNED_SHORT; };
template<> struct TypeInfo<int           > { static constexpr GLenum gl_type = GL_INT           ; };
template<> struct TypeInfo<unsigned int  > { static constexpr GLenum gl_type = GL_UNSIGNED_INT  ; };

}
