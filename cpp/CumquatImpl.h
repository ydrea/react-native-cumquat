#pragma once

#include <CumquatSpecJSI.h>

#include <memory>

namespace facebook::react {

class CumquatImpl
  : public NativeCumquatCxxSpec<CumquatImpl> {
public:
  CumquatImpl(std::shared_ptr<CallInvoker> jsInvoker);

  double multiply(jsi::Runtime& rt, double a, double b);
};

}
