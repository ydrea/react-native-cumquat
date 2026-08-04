import type { ConfigContext, ExpoConfig } from 'expo/config';

const APP_VARIANT = (globalThis as any).process?.env?.APP_VARIANT ?? 'development';

// Atlas:
// const BASE_NAME = 'ATLAS';
// const BASE_ID = 'com.ydrea.atlas';

// Orientation app instead:
const BASE_NAME = 'Cumquat Orientation';
const BASE_ID = 'com.ydrea.cumquatorientation';

const variants = {
  development: {
    name: `${BASE_NAME} (Dev)`,
    suffix: '.dev',
  },
  preview: {
    name: `${BASE_NAME} (Preview)`,
    suffix: '.preview',
  },
} as const;

const variant =
  variants[APP_VARIANT as keyof typeof variants] ?? variants.development;

export default ({ config }: ConfigContext): ExpoConfig => ({
  ...config,
  name: variant.name,
  slug: config.slug ?? "cumquat-orientation",
  android: {
    ...config.android,
    package: `${BASE_ID}${variant.suffix}`,
  },
  ios: {
    ...config.ios,
    bundleIdentifier: `${BASE_ID}${variant.suffix}`,
  },
  runtimeVersion: {
    policy: 'appVersion',
  },
});