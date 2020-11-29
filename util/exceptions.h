#pragma once

#include <stdexcept>
#include <string>
#include "debug.h"

// Base class for exceptions. Can add stack trace here later.
class ExceptionBase {
 public:
  virtual ~ExceptionBase() {}
};

#define EXCEPTION_TYPE(name, base) \
  class name: public base, public ExceptionBase { \
   public: \
    name(const std::string &msg = #name): base(msg) {} \
  };

// Exception for which occurence deserves an investigation.
// Triggers debugger break in constructor.
#define ABNORMAL_EXCEPTION(name, base) \
  class name: public base, public ExceptionBase { \
   public: \
    name(const std::string &msg = #name): base(msg) { \
      MaybeDebugBreak(); \
    } \
  };

EXCEPTION_TYPE(NotImplementedException, std::logic_error);
ABNORMAL_EXCEPTION(GLException, std::runtime_error);
EXCEPTION_TYPE(IOException, std::runtime_error);
EXCEPTION_TYPE(CommandLineArgumentsException, std::runtime_error);
EXCEPTION_TYPE(ShaderCompilationException, std::runtime_error);

#undef EXCEPTION_TYPE
#undef ABNORMAL_EXCEPTION

void LogCurrentException();
