import * as Updates from "expo-updates";
import { Button, Text, View } from "react-native";

export function UpdatePanel() {
  const {
    currentlyRunning,
    isChecking,
    isDownloading,
    isUpdateAvailable,
    isUpdatePending,
    downloadProgress,
    checkError,
    downloadError,
  } = Updates.useUpdates();

  const checkForUpdate = async () => {
    if (!Updates.isEnabled) {
      return;
    }

    await Updates.checkForUpdateAsync();
  };

  const downloadUpdate = async () => {
    await Updates.fetchUpdateAsync();
  };

  const applyUpdate = async () => {
    await Updates.reloadAsync();
  };

  return (
    <View>
      <Text>Channel: {currentlyRunning.channel ?? "development/local"}</Text>

      <Text>
        Runtime: {currentlyRunning.runtimeVersion ?? "unavailable"}
      </Text>

      <Text>
        Source:{" "}
        {currentlyRunning.isEmbeddedLaunch
          ? "embedded build"
          : "EAS update"}
      </Text>

      <Text>
        Update ID: {currentlyRunning.updateId?.slice(0, 8) ?? "embedded"}
      </Text>

      <Button
        title={isChecking ? "Checking…" : "Check for update"}
        disabled={isChecking || !Updates.isEnabled}
        onPress={checkForUpdate}
      />

      {isUpdateAvailable && !isUpdatePending ? (
        <Button
          title={
            isDownloading
              ? `Downloading ${Math.round((downloadProgress ?? 0) * 100)}%`
              : "Download update"
          }
          disabled={isDownloading}
          onPress={downloadUpdate}
        />
      ) : null}

      {isUpdatePending ? (
        <Button title="Restart and apply update" onPress={applyUpdate} />
      ) : null}

      {checkError ? <Text>Check failed: {checkError.message}</Text> : null}
      {downloadError ? (
        <Text>Download failed: {downloadError.message}</Text>
      ) : null}
    </View>
  );
}