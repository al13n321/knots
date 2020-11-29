#pragma once
#include "gl-util/type-info.h"
#include <vector>

namespace GL {

// A "vertex array object" and a collection of "vertex buffer objects".
class VertexArray {
 public:
  struct Attribute {
    GLuint index;
    GLint components;
    GLenum type;
    bool normalized = false;
    size_t stride = 0;
    size_t offset = 0;

    Attribute() = default;
    Attribute(GLuint index, GLint components, GLenum type, size_t stride = 0, size_t offset = 0)
      : index(index), components(components), type(type), stride(stride), offset(offset) {}
  };

  // Initially vertices have no attributes (not even position),
  // so vertex array is pretty useless until you call AddAttribute().
  VertexArray(size_t vertices);
  ~VertexArray();

  VertexArray(const VertexArray& rhs) = delete;
  VertexArray& operator=(const VertexArray& rhs) = delete;

  // Add vertex attribute (e.g. position, texture coordinates).
  // `components` is the number of values per vertex (1, 2, 3 or 4),
  // `vertices` is number of vertices.
  template<typename T>
  void AddAttribute(GLuint index, GLint components, const T* data, bool normalized = false) {
    AddAttribute(index, components, vertices_ * components * sizeof(T), TypeInfo<T>::gl_type, data, normalized);
  }
  void AddAttribute(GLuint index, GLint components, GLint total_bytes, GLenum type, const void* data, bool normalized);

  // A more general interface. Adds one buffer object containing `count` attributes described by `attrs`.
  void AddAttributes(size_t count, const Attribute* attrs, GLint total_bytes, const void* data);

  void Draw();

 private:
  size_t vertices_;
  GLuint vao_;
  std::vector<GLuint> vbos_;
};

}
