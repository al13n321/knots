#include "shader.h"
#include <string>
#include <iostream>
#include <fstream>
#include "util/exceptions.h"
using namespace std;

namespace GL {

static void CompileShader(
    GLint shader, const string &name, const string &text) {
  int str_l = static_cast<int>(text.length());
  const char *c_str = text.c_str();
  glShaderSource(shader, 1, &c_str, &str_l);CHECK_GL_ERROR();
  glCompileShader(shader);CHECK_GL_ERROR();
  GLint ret;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ret);CHECK_GL_ERROR();

  const int maxlen = 100000;
  GLchar buffer[maxlen];
  GLsizei l;
  glGetShaderInfoLog(shader, maxlen, &l, buffer);CHECK_GL_ERROR();

  if (!ret || buffer[0] != '\0') {
    if (ret) {
      cerr << "compiled: " << name << "\n";
    } else {
      cerr << "compilation failed: " << name << "\n";
    }
    cerr << buffer << "\n" << endl;

    if (!ret)
      throw ShaderCompilationException("compilation error in " + name);
  }
}

Shader::Shader(
    const std::string &vert_name,
    const std::string &frag_name,
    const std::string &vert_text,
    const std::string &frag_text) {
  try {
    GLint ret;

    vs_ = glCreateShader(GL_VERTEX_SHADER);CHECK_GL_ERROR();
    CompileShader(vs_, vert_name, vert_text);

    ps_ = glCreateShader(GL_FRAGMENT_SHADER);CHECK_GL_ERROR();
    CompileShader(ps_, frag_name, frag_text);

    program_ = glCreateProgram();CHECK_GL_ERROR();
    glAttachShader(program_, vs_);CHECK_GL_ERROR();
    glAttachShader(program_, ps_);CHECK_GL_ERROR();
    glLinkProgram(program_);CHECK_GL_ERROR();

    glGetProgramiv(program_, GL_LINK_STATUS, &ret);CHECK_GL_ERROR();

    const int maxlen = 100000;
    GLchar buffer[maxlen];
    GLsizei l;
    glGetProgramInfoLog(program_, maxlen, &l, buffer);CHECK_GL_ERROR();
    if (!ret || buffer[0] != '\0'){
      if (ret)
        cerr << "linked: " << vert_name << " and " << frag_name << "\n";
      else
        cerr << "linking failed: " << vert_name << " and " << frag_name << "\n";
      cerr << buffer << "\n" << endl;

      if (!ret)
        throw ShaderCompilationException(
          "linking error in " + vert_name + " and " + frag_name);
    }

    int cnt;
    glGetProgramiv(program_, GL_ACTIVE_UNIFORMS, &cnt);CHECK_GL_ERROR();
    for (int i = 0; i < cnt; ++i) {
      char name[GL_ACTIVE_UNIFORM_MAX_LENGTH];
      GLsizei namelen;
      Uniform uni;
      glGetActiveUniform(
        program_, i, GL_ACTIVE_UNIFORM_MAX_LENGTH,
        &namelen, &uni.size, &uni.type, name);CHECK_GL_ERROR();
      uni.location = glGetUniformLocation(program_, name);CHECK_GL_ERROR();
      uniforms_[name] = uni;
    }
  } catch (...) {
    if (program_) {
      glDetachShader(program_, vs_);CHECK_GL_ERROR();
      glDetachShader(program_, ps_);CHECK_GL_ERROR();
    }
    if (vs_) {
      glDeleteShader(vs_);CHECK_GL_ERROR();
    }
    if (ps_) {
      glDeleteShader(ps_);CHECK_GL_ERROR();
    }
    if (program_) {
      glDeleteProgram(program_);CHECK_GL_ERROR();
    }
    throw;
  }
}
void Shader::Use() {
  glUseProgram(program_);CHECK_GL_ERROR();
}
GLuint Shader::program_id() {
  return program_;
}
void Shader::LogUniforms() {
  cerr << uniforms_.size() << " active uniforms:" << endl;
  for (const auto it: uniforms_) {
    cerr << it.first << " size: " << it.second.size
      << ", type: " << it.second.type << endl;
  }
  cerr << endl;
}

const Shader::Uniform* Shader::GetUniformLocation(
    const std::string &name, GLenum type) {
  return GetUniformLocation(name, type, type);
}

const Shader::Uniform* Shader::GetUniformLocation(
    const std::string &name, GLenum type1, GLenum type2) {
  auto it = uniforms_.find(name); \
  if (it == uniforms_.end()) { \
    if (!uniform_errors_.count(name)) { \
      uniform_errors_.insert(name); \
      std::cerr << "trying to assign a non-existing or unused uniform " << name
        << std::endl;
    }
    return nullptr;
  }
  const Shader::Uniform &res = it->second;
  if (res.type != type1 && res.type != type2) {
    if (!uniform_errors_.count(name)) { \
      uniform_errors_.insert(name); \
      std::cerr << "trying to assign value of wrong type to uniform " << name
        << std::endl;
    }
    return nullptr;
  }

  return &res;
}

void Shader::SetTexture(
  const std::string &name, const Texture2D &texture, int unit
) {
  auto *uni = GetUniformLocation(name, GL_SAMPLER_2D);
  if (uni)
    texture.AssignToUniform(uni->location, unit);
}

void Shader::SetScalar(const std::string &name, double value) {
  auto *uni = GetUniformLocation(name, GL_FLOAT, GL_DOUBLE);
  if (uni) {
    if (uni->type == GL_FLOAT) {
      glUniform1f(uni->location, static_cast<float>(value)); CHECK_GL_ERROR();
    } else {
      glUniform1d(uni->location, value); CHECK_GL_ERROR();
    }
  }
}

void Shader::SetVec2(const std::string &name, dvec2 value) {
  auto *uni = GetUniformLocation(name, GL_FLOAT_VEC2, GL_DOUBLE_VEC2);
  if (uni) {
    if (uni->type == GL_FLOAT_VEC2) {
      glUniform2f(uni->location, (float)value.x, (float)value.y); CHECK_GL_ERROR();
    } else {
      glUniform2d(uni->location, value.x, value.y); CHECK_GL_ERROR();
    }
  }
}

void Shader::SetVec3(const std::string &name, dvec3 value) {
  auto *uni = GetUniformLocation(name, GL_FLOAT_VEC3, GL_DOUBLE_VEC3);
  if (uni) {
    if (uni->type == GL_FLOAT_VEC3) {
      glUniform3f(uni->location, (float)value.x, (float)value.y, (float)value.z); CHECK_GL_ERROR();
    } else {
      glUniform3d(uni->location, value.x, value.y, value.z); CHECK_GL_ERROR();
    }
  }
}

void Shader::SetVec4(const std::string &name, dvec4 value) {
  auto *uni = GetUniformLocation(name, GL_FLOAT_VEC4, GL_DOUBLE_VEC4);
  if (uni) {
    if (uni->type == GL_FLOAT_VEC4) {
      glUniform4f(uni->location, (float)value.x, (float)value.y, (float)value.z, (float)value.w);
        CHECK_GL_ERROR();
    } else {
      glUniform4d(uni->location, value.x, value.y, value.z, value.w);
        CHECK_GL_ERROR();
    }
  }
}

void Shader::SetMat4(const std::string &name, const fmat4 &value) {
  auto *uni = GetUniformLocation(name, GL_FLOAT_MAT4);
  if (uni)
    glUniformMatrix4fv(uni->location, 1, true, value.m);
}

Shader::~Shader() {
  glDetachShader(program_, vs_);
  glDetachShader(program_, ps_);
  glDeleteShader(vs_);
  glDeleteShader(ps_);
  glDeleteProgram(program_);
}

}
