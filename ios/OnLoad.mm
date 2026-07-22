#import <Foundation/Foundation.h>
#import "CumquatImpl.h"
#import <ReactCommon/CxxTurboModuleUtils.h>

@interface CumquatOnLoad : NSObject
@end

@implementation CumquatOnLoad

using namespace facebook::react;

+ (void)load
{
  registerCxxModuleToGlobalModuleMap(
    std::string(CumquatImpl::kModuleName),
    [](std::shared_ptr<CallInvoker> jsInvoker) {
      return std::make_shared<CumquatImpl>(jsInvoker);
    }
  );
}

@end
