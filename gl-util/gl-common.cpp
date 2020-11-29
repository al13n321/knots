#include "gl-common.h"
#include <iostream>
#include <map>
#include <mutex>
#include "util/exceptions.h"

namespace GL {

static std::map<GLenum, std::string> names = {
  {GL_NO_ERROR, "GL_NO_ERROR. No error has been recorded. The value of this symbolic constant is guaranteed to be 0."},
  {GL_INVALID_ENUM, "GL_INVALID_ENUM. An unacceptable value is specified for an enumerated argument. The offending command is ignored and has no other side effect than to set the error flag."},
  {GL_INVALID_VALUE, "GL_INVALID_VALUE. A numeric argument is out of range. The offending command is ignored and has no other side effect than to set the error flag."},
  {GL_INVALID_OPERATION, "GL_INVALID_OPERATION. The specified operation is not allowed in the current state. The offending command is ignored and has no other side effect than to set the error flag."},
  {GL_INVALID_FRAMEBUFFER_OPERATION, "GL_INVALID_FRAMEBUFFER_OPERATION. The framebuffer object is not complete. The offending command is ignored and has no other side effect than to set the error flag."},
  {GL_OUT_OF_MEMORY, "GL_OUT_OF_MEMORY. There is not enough memory left to execute the command. The state of the GL is undefined, except for the state of the error flags, after this error is recorded."},
#ifdef GL_STACK_UNDERFLOW
  {GL_STACK_UNDERFLOW, "GL_STACK_UNDERFLOW. An attempt has been made to perform an operation that would cause an internal stack to underflow."},
#endif
#ifdef GL_STACK_OVERFLOW
  {GL_STACK_OVERFLOW, "GL_STACK_OVERFLOW. An attempt has been made to perform an operation that would cause an internal stack to overflow."},
#endif
};

void ThrowIfError(const char *file, int line) {
  GLenum err = glGetError();
  if (err != GL_NO_ERROR)
    throw GLException(std::string("GL error at ")
      + file + ":" + std::to_string(line) + " " + names[err]);
}

bool LogIfError(const char *file, int line) {
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    std::cerr << std::string("GL error at ")
      << file << ":" << std::to_string(line) << " " << names[err] << std::endl;
    return true;
  }

  return false;
}

void LogInfo() {
  const GLubyte *str;
  
  str = glGetString(GL_VENDOR); CHECK_GL_ERROR();
  std::cerr << "GL vendor: " << str << std::endl;

  str = glGetString(GL_RENDERER); CHECK_GL_ERROR();
  std::cerr << "GL renderer: " << str << std::endl;

  str = glGetString(GL_VERSION); CHECK_GL_ERROR();
  std::cerr << "GL version: " << str << std::endl;

  str = glGetString(GL_SHADING_LANGUAGE_VERSION); CHECK_GL_ERROR();
  std::cerr << "GLSL: " << str << std::endl;

  std::cerr << std::endl;
}

void InitGl3wIfNeeded() {
  static std::once_flag flag;
  std::call_once(flag, [](){
    if (gl3wInit())
      throw GLException("failed to initialize gl3w");
  });
}

}
