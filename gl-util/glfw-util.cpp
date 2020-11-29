#include "glfw-util.h"
#include <iostream>

#ifdef WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#include <GLFW/glfw3native.h>
#endif

namespace glfw {

#ifdef _WIN32
HWND Window::GetHWND() {
  return glfwGetWin32Window(window);
}
#endif

void Window::Focus() {
#ifdef _WIN32
  ShowWindow(GetHWND(), SW_SHOWNORMAL);
  SetFocus(GetHWND());
#elif defined(USE_OVR)
  std::cerr
    << "Window::Focus() is not implemented for this platform. "
    << "Consider implementing it to avoid the crappy user experience "
    << "you are currently having." << std::endl;
#endif
}

}
