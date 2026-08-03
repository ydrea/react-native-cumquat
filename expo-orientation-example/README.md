# Cumquat Orientation Visualizer

An Expo development-build app that renders the orientation quaternion used by
`react-native-cumquat` as a literal 3D phone with device and ENU axes.

The visualizer is Android-first because Cumquat owns the native Android
`TYPE_GAME_ROTATION_VECTOR` sensor. It deliberately does not read a second JS
sensor stream. The screen is portrait-locked so rotating the physical phone
does not rotate the diagnostic UI.

## Run against the local package

From this directory:

```bash
npm install
npx expo prebuild --platform android --clean
npx expo run:android
```

Expo Go cannot load Cumquat's TurboModule. Use the development build installed
by `expo run:android`; for later JS changes run:

```bash
npx expo start --dev-client --clear
```

The `RAW` mode is the exact frame quaternion. `SMOOTH` applies display-only
slerp and never changes Cumquat's sensor or projection state.
