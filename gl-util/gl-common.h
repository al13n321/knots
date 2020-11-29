#define NOMINMAX
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

namespace GL {

// Throw if glGetError() returns non-success.
void ThrowIfError(const char *file, int line);

// Returns true if glError() returns non-success.
bool LogIfError(const char *file, int line);

#define CHECK_GL_ERROR() GL::ThrowIfError(__FILE__, __LINE__)
#define SOFT_CHECK_GL_ERROR() GL::LogIfError(__FILE__, __LINE__)

// Log OpenGL vendor, version, extensions etc.
void LogInfo();

// The first time it's called, calls gl3wInit().
// Call after creating the first OpenGL context.
// As a hack we rely on function addresses to be the same for all OpenGL
// contexts. On Windows it's true only for contexts with same pixel format.
// If it becomes a problem, migrate to glbinding.
void InitGl3wIfNeeded();

}
