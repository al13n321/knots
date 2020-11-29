#pragma once

#include "gl-common.h"
#include "texture2d.h"
#include "util/vec.h"
#include "util/mat.h"
#include <string>
#include <map>
#include <set>

namespace GL {

const char * const kDefaultShaderAttribnames[]={"inScreenPos","inCanvasPos"};

#define SHADER_INFO_LOG

class Shader {
public:
  Shader(
    const std::string &vert_name, // For logging and exception messages only.
    const std::string &frag_name, // For logging and exception messages only.
    const std::string &vert_text,
    const std::string &frag_text);
  ~Shader();

  // Setting uniforms. Float and double are interchangeable.
  // If the uniform doesn't exist or the type is incorrect, logs and doesn't
  // throw. Logs only once for each name to avoid flooding the log.
  void SetTexture(const std::string &name, const Texture2D &texture, int unit);
  void SetScalar(const std::string &name, double value);
  void SetVec2(const std::string &name, dvec2 value);
  void SetVec3(const std::string &name, dvec3 value);
  void SetVec4(const std::string &name, dvec4 value);
  void SetMat4(const std::string &name, const fmat4 &value);

  void Use();
  GLuint program_id();
  void LogUniforms(); // writes information about all active uniforms to stdout
private:
  struct Uniform {
    GLint location;
    GLint size;
    GLenum type;
  };

  GLuint vs_{};
  GLuint ps_{};
  GLuint program_{};
  std::map<std::string, Uniform> uniforms_;

  // For what uniforms we already logged an error.
  std::set<std::string> uniform_errors_;

  const Uniform* GetUniformLocation(const std::string &name, GLenum type);
  const Uniform* GetUniformLocation(const std::string &name, GLenum type1, GLenum type2);
};

}
