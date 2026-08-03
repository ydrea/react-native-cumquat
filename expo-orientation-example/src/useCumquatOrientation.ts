import { useEffect, useRef, useState } from 'react';
import { Platform } from 'react-native';
import {
  CumquatEngine,
  type FrameOrientation,
  type Quaternion,
} from 'react-native-cumquat';

import { slerp } from './orientationMath';

type OrientationState = {
  orientation: FrameOrientation | null;
  rawQuaternion: Quaternion | null;
  sequence: number;
  fps: number;
  nativeVersion: string;
  status: 'starting' | 'calibrating' | 'ready' | 'unsupported' | 'error';
  error: string | null;
};

const INITIAL_STATE: OrientationState = {
  orientation: null,
  rawQuaternion: null,
  sequence: 0,
  fps: 0,
  nativeVersion: 'unknown',
  status: 'starting',
  error: null,
};

export function useCumquatOrientation(
  viewportWidth: number,
  viewportHeight: number,
  smoothingEnabled: boolean
): OrientationState {
  const [state, setState] = useState(INITIAL_STATE);
  const displayedQuaternion = useRef<Quaternion | null>(null);

  useEffect(() => {
    if (Platform.OS !== 'android') {
      setState((current) => ({ ...current, status: 'unsupported' }));
      return;
    }

    let engine: CumquatEngine | null = null;
    let animationFrame = 0;
    let active = true;
    let previousFrameTime = 0;
    let fpsWindowStarted = performance.now();
    let fpsFrameCount = 0;
    let measuredFps = 0;

    try {
      engine = CumquatEngine.create();
      engine.initialize([]);
      engine.setViewState({
        horizontalFovDegrees: 90,
        minDistanceMeters: 0,
        maxDistanceMeters: 135_000,
      });
      setState((current) => ({
        ...current,
        nativeVersion: CumquatEngine.getNativeVersion(),
        status: 'calibrating',
      }));
    } catch (error) {
      setState((current) => ({
        ...current,
        status: 'error',
        error: error instanceof Error ? error.message : String(error),
      }));
      return;
    }

    const tick = (now: number) => {
      if (!active || !engine) return;
      animationFrame = requestAnimationFrame(tick);

      // Updating React at ~30 Hz keeps the diagnostic UI responsive while the
      // native game-rotation sensor continues sampling at 60 Hz.
      if (now - previousFrameTime < 33) return;
      previousFrameTime = now;

      try {
        engine.update({
          timestampNs: Date.now() * 1_000_000,
          location: { latitude: 0, longitude: 0, altitude: 0 },
          headingDegrees: 0,
          pitchDegrees: 0,
          rollDegrees: 0,
          viewportWidth: Math.max(1, viewportWidth),
          viewportHeight: Math.max(1, viewportHeight),
        });

        const frame = engine.getFrame();
        const rawQuaternion = frame.orientation?.quaternion ?? null;
        let orientation = frame.orientation;

        if (orientation && smoothingEnabled && displayedQuaternion.current) {
          orientation = {
            ...orientation,
            quaternion: slerp(
              displayedQuaternion.current,
              rawQuaternion!,
              0.22
            ),
          };
        }
        displayedQuaternion.current = orientation?.quaternion ?? null;

        fpsFrameCount += 1;
        const fpsElapsed = now - fpsWindowStarted;
        if (fpsElapsed >= 1_000) {
          measuredFps = (fpsFrameCount * 1_000) / fpsElapsed;
          fpsFrameCount = 0;
          fpsWindowStarted = now;
        }

        setState((current) => ({
          ...current,
          orientation,
          rawQuaternion,
          sequence: frame.sequence,
          fps: measuredFps,
          status: orientation ? 'ready' : 'calibrating',
          error: null,
        }));
      } catch (error) {
        setState((current) => ({
          ...current,
          status: 'error',
          error: error instanceof Error ? error.message : String(error),
        }));
      }
    };

    animationFrame = requestAnimationFrame(tick);
    return () => {
      active = false;
      cancelAnimationFrame(animationFrame);
      engine?.dispose();
    };
  }, [smoothingEnabled, viewportHeight, viewportWidth]);

  return state;
}
