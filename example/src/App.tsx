import { StyleSheet, Text, View } from 'react-native';
import { getCumquatNativeVersion } from 'react-native-cumquat';

export default function App() {
  const nativeVersion = getCumquatNativeVersion();

  return (
    <View style={styles.container}>
      <Text style={styles.title}>react-native-cumquat</Text>
      <Text>Native engine: {nativeVersion}</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
    gap: 8,
  },
  title: {
    fontSize: 20,
    fontWeight: '600',
  },
});
