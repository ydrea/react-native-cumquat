import { useEffect, useState } from 'react';
import {
  Pressable,
  ScrollView,
  StatusBar,
  StyleSheet,
  Text,
  useWindowDimensions,
  View,
} from 'react-native';

import { PhoneScene } from './PhoneScene';
import { cameraBearing } from './orientationMath';
import { useCumquatOrientation } from './useCumquatOrientation';
import { UpdatePanel } from './UpdatePanel';

function number(value: number | undefined) {
  return value === undefined ? '—' : value.toFixed(5);
}

export default function App() {
  const { width, height } = useWindowDimensions();
  const [smoothingEnabled, setSmoothingEnabled] = useState(false);
  const sceneHeight = Math.max(240, height * 0.4);
  const state = useCumquatOrientation(width, sceneHeight, smoothingEnabled);
  const q = state.rawQuaternion;
  const bearing = cameraBearing(state.orientation);

  useEffect(() => {
    console.log('state', state);
  }, [state]);

  return (
    <ScrollView style={styles.safeArea}>
      <StatusBar barStyle="light-content" backgroundColor="#07101e" />
      <View style={styles.header}>
        <View>
          <Text style={styles.eyebrow}>REACT-NATIVE-CUMQUAT</Text>
          <Text style={styles.title}>Orientation</Text>
        </View>
        <View style={[styles.status, styles[`status_${state.status}`]]}>
          <View style={styles.statusDot} />
          <Text style={styles.statusText}>{state.status.toUpperCase()}</Text>
        </View>
      </View>

      <View style={[styles.scene, { height: sceneHeight }]}>
        <PhoneScene
          orientation={state.orientation}
          width={width - 32}
          height={sceneHeight}
        />
        {state.status === 'calibrating' && (
          <View style={styles.sceneMessage}>
            <Text style={styles.sceneMessageTitle}>Hold the phone steady</Text>
            <Text style={styles.sceneMessageText}>
              Cumquat is pairing its absolute and game-rotation sensors.
            </Text>
          </View>
        )}
      </View>

      <View style={styles.panel}>
        <View style={styles.panelHeader}>
          <Text style={styles.panelTitle}>FRAME {state.sequence}</Text>
          <Pressable
            accessibilityRole="button"
            onPress={() => setSmoothingEnabled((enabled) => !enabled)}
            style={[styles.toggle, smoothingEnabled && styles.toggleActive]}
          >
            <Text style={styles.toggleText}>
              {smoothingEnabled ? 'SMOOTH' : 'RAW'}
            </Text>
          </Pressable>
        </View>

        <View style={styles.quaternionRow}>
          {(['x', 'y', 'z', 'w'] as const).map((component) => (
            <View key={component} style={styles.valueCell}>
              <Text style={styles.valueLabel}>{component.toUpperCase()}</Text>
              <Text style={styles.value}>{number(q?.[component])}</Text>
            </View>
          ))}
        </View>

        <View style={styles.metaRow}>
          <Text style={styles.metaLabel}>Camera bearing</Text>
          <Text style={styles.metaValue}>
            {bearing === null ? '—' : `${Math.round(bearing)}°`}
          </Text>
        </View>
        <View style={styles.metaRow}>
          <Text style={styles.metaLabel}>Convention</Text>
          <Text style={styles.metaValue}>
            {state.orientation?.convention ?? 'not ready'}
          </Text>
        </View>
        <View style={styles.metaRow}>
          <Text style={styles.metaLabel}>Visualizer rate</Text>
          <Text style={styles.metaValue}>{state.fps.toFixed(1)} fps</Text>
        </View>
        <View style={styles.metaRow}>
          <Text style={styles.metaLabel}>Native engine</Text>
          <Text style={styles.metaValue}>{state.nativeVersion}</Text>
        </View>

        {state.status === 'unsupported' && (
          <Text style={styles.warning}>
            This visualizer currently reads Cumquat’s native game-rotation
            sensor on Android. The package API also supports non-Android frames
            when an app supplies its synchronized DeviceMotion/heading pair.
          </Text>
        )}
        {state.error && <Text style={styles.error}>{state.error}</Text>}

        <View style={styles.updateSection}>
          <UpdatePanel />
        </View>
      </View>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: '#07101e' },
  header: {
    paddingHorizontal: 20,
    paddingTop: 14,
    paddingBottom: 4,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  eyebrow: {
    color: '#6f86a8',
    fontSize: 10,
    letterSpacing: 2.2,
    fontWeight: '700',
  },
  title: { color: '#f5f8ff', fontSize: 11, lineHeight: 36, fontWeight: '300' },
  status: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 7,
    paddingHorizontal: 11,
    paddingVertical: 7,
    borderRadius: 999,
    backgroundColor: '#3a2b14',
  },
  status_ready: { backgroundColor: '#143a2c' },
  status_error: { backgroundColor: '#4a2028' },
  status_unsupported: { backgroundColor: '#352b4b' },
  status_starting: {},
  status_calibrating: {},
  statusDot: {
    width: 6,
    height: 6,
    borderRadius: 3,
    backgroundColor: '#ffd27a',
  },
  statusText: {
    color: '#f6e5bc',
    fontSize: 10,
    letterSpacing: 1.1,
    fontWeight: '800',
  },
  scene: {
    marginHorizontal: 16,
    marginTop: 8,
    borderWidth: 1,
    borderColor: '#172943',
    borderRadius: 22,
    overflow: 'hidden',
    backgroundColor: '#091526',
  },
  sceneMessage: {
    position: 'absolute',
    left: 16,
    right: 16,
    bottom: 14,
    alignItems: 'center',
  },
  sceneMessageTitle: { color: '#f3f6fd', fontSize: 12, fontWeight: '700' },
  sceneMessageText: {
    color: '#8294af',
    fontSize: 11,
    marginTop: 3,
    textAlign: 'center',
  },
  panel: {
    margin: 16,
    marginTop: 2,
    padding: 16,
    borderRadius: 18,
    backgroundColor: '#0d1a2c',
    borderWidth: 1,
    borderColor: '#182b45',
  },
  panelHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: 2,
  },
  panelTitle: {
    color: '#8294af',
    fontSize: 11,
    letterSpacing: 1.5,
    fontWeight: '800',
  },
  toggle: {
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#30415c',
    paddingHorizontal: 10,
    paddingVertical: 6,
  },
  toggleActive: { backgroundColor: '#183a4a', borderColor: '#3b91aa' },
  toggleText: {
    color: '#c6d3e8',
    fontSize: 10,
    letterSpacing: 1,
    fontWeight: '800',
  },
  quaternionRow: { flexDirection: 'row', gap: 7, marginBottom: 14 },
  valueCell: {
    flex: 1,
    backgroundColor: '#091526',
    padding: 9,
    borderRadius: 10,
  },
  valueLabel: { color: '#607493', fontSize: 9, fontWeight: '800' },
  value: {
    color: '#e9f0fc',
    fontSize: 12,
    marginTop: 4,
    fontVariant: ['tabular-nums'],
  },
  metaRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    paddingVertical: 4,
  },
  metaLabel: { color: '#71839f', fontSize: 12 },
  metaValue: { color: '#c8d4e7', fontSize: 12, fontVariant: ['tabular-nums'] },
  updateSection: {
    marginTop: 16,
    paddingTop: 16,
    borderTopWidth: 1,
    borderTopColor: '#182b45',
  },
  warning: { color: '#b6a7da', fontSize: 11, lineHeight: 16, marginTop: 2 },
  error: { color: '#ff8b98', fontSize: 11, lineHeight: 16, marginTop: 2 },
});
