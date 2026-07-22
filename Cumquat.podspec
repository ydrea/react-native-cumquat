require "json"

package = JSON.parse(File.read(File.join(__dir__, "package.json")))

Pod::Spec.new do |s|
  s.name         = "Cumquat"
  s.version      = package["version"]
  s.summary      = package["description"]
  s.description  = package["description"]
  s.homepage     = package["homepage"]
  s.license      = package["license"]
  s.authors      = package["author"]

  s.platforms    = { :ios => "15.1" }
  s.source       = {
    :git => "https://github.com/ydrea/react-native-cumquat.git",
    :tag => "v#{s.version}",
  }

  s.source_files =
    "ios/**/*.{h,m,mm}",
    "cpp/**/*.{hpp,cpp,c,h}",
    "ios/generated/*.{h,cpp,mm}"
  s.private_header_files = "ios/**/*.h"
  s.header_mappings_dir = "."

  s.pod_target_xcconfig = {
    "CLANG_CXX_LANGUAGE_STANDARD" => "c++20",
  }

  install_modules_dependencies(s)
end
