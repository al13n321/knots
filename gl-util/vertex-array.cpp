#pragma warning(disable:4267) // implicit conversions between integer types
#include "gl-util/vertex-array.h"

namespace GL {

VertexArray::VertexArray(size_t vertices): vertices_(vertices) {
  glGenVertexArrays(1, &vao_);CHECK_GL_ERROR();
}
VertexArray::~VertexArray() {
  glDeleteVertexArrays(1, &vao_);
  if (!vbos_.empty()) {
    glDeleteBuffers(vbos_.size(), &vbos_[0]);
  }
  CHECK_GL_ERROR();
}

void VertexArray::AddAttribute(GLuint index, GLint components, GLint total_bytes, GLenum type, const void* data, bool normalized) {
  Attribute a(index, components, type);
  a.normalized = normalized;
  AddAttributes(1, &a, total_bytes, data);
}

void VertexArray::AddAttributes(size_t count, const Attribute* attrs, GLint total_bytes, const void* data) {
  glBindVertexArray(vao_);CHECK_GL_ERROR();
  GLuint vbo;
  glGenBuffers(1, &vbo);CHECK_GL_ERROR();
  vbos_.push_back(vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);CHECK_GL_ERROR();
  glBufferData(GL_ARRAY_BUFFER, total_bytes, data, GL_STATIC_DRAW);CHECK_GL_ERROR();
  for (size_t i = 0; i < count; ++i) {
    const Attribute& a = attrs[i];
    glEnableVertexAttribArray(a.index);CHECK_GL_ERROR();
    glVertexAttribPointer(a.index, a.components, a.type, a.normalized, a.stride, (void*)a.offset);CHECK_GL_ERROR();
  }
}

void VertexArray::Draw() {
  glBindVertexArray(vao_);CHECK_GL_ERROR();
  glDrawArrays(GL_TRIANGLES, 0, vertices_);CHECK_GL_ERROR();
  glBindVertexArray(0);CHECK_GL_ERROR();
}

}
