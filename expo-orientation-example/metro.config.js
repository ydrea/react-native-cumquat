const path = require('node:path');
const { getDefaultConfig } = require('expo/metro-config');

const projectRoot = __dirname;
const packageRoot = path.resolve(projectRoot, '..');
const config = getDefaultConfig(projectRoot);

config.watchFolders = [...config.watchFolders, packageRoot];
config.resolver.unstable_conditionNames = [
  'react-native-cumquat-source',
  ...config.resolver.unstable_conditionNames,
];

module.exports = config;
