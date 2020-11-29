#pragma once

#include "gl-common.h"
#include "util/vec.h"

namespace GL {

class Texture2D{
 public:
  Texture2D(const Texture2D &rhs) = delete;
  Texture2D& operator=(const Texture2D &rhs) = delete;

  Texture2D(ivec2 size, GLint internalFormat, GLint filter = GL_NEAREST);
  Texture2D(ivec2 size, GLint internalFormat, GLenum format, GLenum type,
            const GLvoid *data, GLint filter = GL_NEAREST);
  ~Texture2D();

  GLuint name() { return name_; } // Not const because can render to it.
  ivec2 size() const { return size_; }
  void AssignToUniform(GLint uniform, int unit) const;
  void SetFilter(GLint filter);
  void SetPixels(ivec2 pos, ivec2 size, GLenum format, GLenum type, void *data);
  void SetPixels(GLenum format, GLenum type, void *data);
 protected:
  GLuint name_;
  ivec2 size_;
};

}
