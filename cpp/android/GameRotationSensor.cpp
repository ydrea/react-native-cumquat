#include "GameRotationSensor.h"
#include "RotationAlignment.h"

#ifdef __ANDROID__

#include <android/looper.h>
#include <android/sensor.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

namespace cumquat::android {
namespace {
constexpr int kSensorLooperId = 1;
constexpr int kUpdatePeriodMicroseconds = 16667;
}

struct GameRotationSensor::Impl {
  std::atomic<bool> running{true};
  std::atomic<bool> available{false};
  mutable std::mutex mutex;
  RotationAlignment alignment;
  TimedOrientation latestAbsolute;
  TimedOrientation latestGame;
  std::int64_t latestMagneticFieldTimestampNs{0};
  std::int64_t lastPairedAbsoluteTimestampNs{0};
  std::int64_t lastPairedGameTimestampNs{0};
  double latestMagneticFieldMicrotesla{0.0};
  bool hasAbsolute{false};
  bool hasGame{false};
  bool hasMagneticField{false};
  std::thread worker;

  Impl() : worker([this] { run(); }) {}

  ~Impl() {
    running.store(false);
    if (worker.joinable()) worker.join();
  }

  void run() {
    ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ASensorManager* manager = ASensorManager_getInstance();
    if (!looper || !manager) return;

    const ASensor* gameSensor = ASensorManager_getDefaultSensor(
        manager,
        ASENSOR_TYPE_GAME_ROTATION_VECTOR);
    const ASensor* absoluteSensor = ASensorManager_getDefaultSensor(
        manager,
        ASENSOR_TYPE_ROTATION_VECTOR);
    if (!absoluteSensor) {
      absoluteSensor = ASensorManager_getDefaultSensor(
          manager,
          ASENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR);
    }
    const ASensor* magneticFieldSensor = ASensorManager_getDefaultSensor(
        manager,
        ASENSOR_TYPE_MAGNETIC_FIELD);
    if (!gameSensor || !absoluteSensor) return;

    ASensorEventQueue* queue = ASensorManager_createEventQueue(
        manager,
        looper,
        kSensorLooperId,
        nullptr,
        nullptr);
    if (!queue) return;

    if (ASensorEventQueue_enableSensor(queue, gameSensor) < 0 ||
        ASensorEventQueue_enableSensor(queue, absoluteSensor) < 0) {
      ASensorManager_destroyEventQueue(manager, queue);
      return;
    }

    ASensorEventQueue_setEventRate(
        queue,
        gameSensor,
        kUpdatePeriodMicroseconds);
    ASensorEventQueue_setEventRate(
        queue,
        absoluteSensor,
        kUpdatePeriodMicroseconds);
    if (magneticFieldSensor &&
        ASensorEventQueue_enableSensor(queue, magneticFieldSensor) >= 0) {
      ASensorEventQueue_setEventRate(
          queue,
          magneticFieldSensor,
          kUpdatePeriodMicroseconds);
    } else {
      magneticFieldSensor = nullptr;
    }
    available.store(true);

    bool calibrationSensorsDisabled = false;
    while (running.load()) {
      const int ident = ALooper_pollOnce(100, nullptr, nullptr, nullptr);
      if (ident != kSensorLooperId) continue;

      ASensorEvent event;
      while (ASensorEventQueue_getEvents(queue, &event, 1) > 0) {
        std::lock_guard lock(mutex);

        if (event.type == ASENSOR_TYPE_MAGNETIC_FIELD) {
          latestMagneticFieldMicrotesla = std::hypot(
              std::hypot(
                  static_cast<double>(event.data[0]),
                  static_cast<double>(event.data[1])),
              static_cast<double>(event.data[2]));
          latestMagneticFieldTimestampNs = event.timestamp;
          hasMagneticField = true;
          continue;
        }

        if (event.type != ASENSOR_TYPE_GAME_ROTATION_VECTOR &&
            event.type != ASENSOR_TYPE_ROTATION_VECTOR &&
            event.type != ASENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR) {
          continue;
        }

        Quaternion orientation{
            static_cast<double>(event.data[0]),
            static_cast<double>(event.data[1]),
            static_cast<double>(event.data[2]),
            0.0,
        };
        const double vectorLengthSquared =
            orientation.x * orientation.x +
            orientation.y * orientation.y +
            orientation.z * orientation.z;
        orientation.w =
            std::sqrt(std::max(0.0, 1.0 - vectorLengthSquared));
        orientation = normalizedQuaternion(orientation);

        if (event.type == ASENSOR_TYPE_GAME_ROTATION_VECTOR) {
          latestGame = {orientation, event.timestamp};
          hasGame = true;
        } else {
          latestAbsolute = {orientation, event.timestamp};
          hasAbsolute = true;
        }

        if (alignment.isCalibrated() || !hasAbsolute || !hasGame) continue;
        if (latestAbsolute.timestampNs == lastPairedAbsoluteTimestampNs ||
            latestGame.timestampNs == lastPairedGameTimestampNs) {
          continue;
        }

        std::optional<double> magneticField;
        constexpr std::int64_t kMaximumFieldAgeNs = 250'000'000;
        const std::int64_t fieldAge =
            latestGame.timestampNs >= latestMagneticFieldTimestampNs
            ? latestGame.timestampNs - latestMagneticFieldTimestampNs
            : latestMagneticFieldTimestampNs - latestGame.timestampNs;
        if (hasMagneticField && fieldAge <= kMaximumFieldAgeNs) {
          magneticField = latestMagneticFieldMicrotesla;
        }

        lastPairedAbsoluteTimestampNs = latestAbsolute.timestampNs;
        lastPairedGameTimestampNs = latestGame.timestampNs;
        if (alignment.addPair(latestAbsolute, latestGame, magneticField) &&
            !calibrationSensorsDisabled) {
          ASensorEventQueue_disableSensor(queue, absoluteSensor);
          if (magneticFieldSensor) {
            ASensorEventQueue_disableSensor(queue, magneticFieldSensor);
          }
          calibrationSensorsDisabled = true;
        }
      }
    }

    available.store(false);
    ASensorEventQueue_disableSensor(queue, gameSensor);
    if (!calibrationSensorsDisabled) {
      ASensorEventQueue_disableSensor(queue, absoluteSensor);
      if (magneticFieldSensor) {
        ASensorEventQueue_disableSensor(queue, magneticFieldSensor);
      }
    }
    ASensorManager_destroyEventQueue(manager, queue);
  }
};

GameRotationSensor::GameRotationSensor() : impl_(std::make_unique<Impl>()) {}
GameRotationSensor::~GameRotationSensor() = default;

bool GameRotationSensor::latest(Quaternion& orientation) const {
  std::lock_guard lock(impl_->mutex);
  if (!impl_->alignment.isCalibrated() || !impl_->hasGame) return false;
  orientation =
      impl_->alignment.alignGameOrientation(impl_->latestGame.orientation);
  return true;
}

bool GameRotationSensor::isAvailable() const {
  return impl_->available.load();
}

bool GameRotationSensor::isCalibrated() const {
  std::lock_guard lock(impl_->mutex);
  return impl_->alignment.isCalibrated();
}

} // namespace cumquat::android

#else

namespace cumquat::android {

struct GameRotationSensor::Impl {};

GameRotationSensor::GameRotationSensor() : impl_(std::make_unique<Impl>()) {}
GameRotationSensor::~GameRotationSensor() = default;

bool GameRotationSensor::latest(Quaternion&) const {
  return false;
}

bool GameRotationSensor::isAvailable() const {
  return false;
}

bool GameRotationSensor::isCalibrated() const {
  return false;
}

} // namespace cumquat::android

#endif
