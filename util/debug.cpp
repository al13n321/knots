#include "debug.h"

#ifdef _WIN32
#include <intrin.h>
#endif

void MaybeDebugBreak() {
#ifndef NDEBUG
  #ifdef _WIN32
    __debugbreak();
  #endif
#endif
}

