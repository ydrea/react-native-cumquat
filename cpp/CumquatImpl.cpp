#include "CumquatImpl.h"

namespace facebook::react {

CumquatImpl::CumquatImpl(
  std::shared_ptr<CallInvoker> jsInvoker
)
  : NativeCumquatCxxSpec(std::move(jsInvoker)) {}

double CumquatImpl::multiply(
  jsi::Runtime& rt,
  double a,
  double b
) {
  return a * b;
}

}
