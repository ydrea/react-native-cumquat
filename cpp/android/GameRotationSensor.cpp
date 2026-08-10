#include "GameRotationSensor.h"
#include "RotationAlignment.h"

#ifdef __ANDROID__

#include <android/looper.h>
#include <android/sensor.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <optional>
#include <thread>

namespace cumquat::android
{
  namespace
  {
    constexpr int kSensorLooperId = 1;
    constexpr int kUpdatePeriodMicroseconds = 16667;

    Quaternion eventQuaternion(const ASensorEvent &event)
    {
      Quaternion orientation{
          static_cast<double>(event.data[0]),
          static_cast<double>(event.data[1]),
          static_cast<double>(event.data[2]),
          0.0,
      };

      // Android's scalar component is optional. Older devices such as the
      // Galaxy S7 can leave event.data[3] at zero even when the true scalar is
      // not zero, so derive it from the mandated vector components.
      const double vectorLengthSquared =
          orientation.x * orientation.x +
          orientation.y * orientation.y +
          orientation.z * orientation.z;
      orientation.w =
          std::sqrt(std::max(0.0, 1.0 - vectorLengthSquared));

      return normalizedQuaternion(orientation);
    }
  }

  struct GameRotationSensor::Impl
  {
    std::atomic<bool> running{true};
    std::atomic<bool> available{false};
    mutable std::mutex mutex;
    RotationAlignment alignment;
    TimedOrientation latestGame;
    TimedOrientation latestAbsolute;
    std::optional<double> magneticFieldMicrotesla;
    Quaternion latestAligned;
    bool hasGame{false};
    bool hasAbsolute{false};
    bool hasAligned{false};
    std::thread worker;

    Impl() : worker([this]
                    { run(); }) {}

    ~Impl()
    {
      running.store(false);
      if (worker.joinable())
        worker.join();
    }

    void updateAlignmentLocked()
    {
      if (!hasGame || !hasAbsolute)
        return;

      alignment.addPair(
          latestAbsolute,
          latestGame,
          magneticFieldMicrotesla);

      if (alignment.isCalibrated())
      {
        latestAligned =
            alignment.alignGameOrientation(latestGame.orientation);
        hasAligned = true;
      }
    }

    void run()
    {
      ALooper *looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
      ASensorManager *manager = ASensorManager_getInstance();
      if (!looper || !manager)
        return;

      const ASensor *gameSensor = ASensorManager_getDefaultSensor(
          manager,
          ASENSOR_TYPE_GAME_ROTATION_VECTOR);
      const ASensor *absoluteSensor = ASensorManager_getDefaultSensor(
          manager,
          ASENSOR_TYPE_ROTATION_VECTOR);
      const ASensor *magneticSensor = ASensorManager_getDefaultSensor(
          manager,
          ASENSOR_TYPE_MAGNETIC_FIELD);
      if (!gameSensor || !absoluteSensor)
        return;

      ASensorEventQueue *queue = ASensorManager_createEventQueue(
          manager,
          looper,
          kSensorLooperId,
          nullptr,
          nullptr);
      if (!queue)
        return;

      const bool gameEnabled =
          ASensorEventQueue_enableSensor(queue, gameSensor) >= 0;
      const bool absoluteEnabled =
          ASensorEventQueue_enableSensor(queue, absoluteSensor) >= 0;
      const bool magneticEnabled =
          magneticSensor &&
          ASensorEventQueue_enableSensor(queue, magneticSensor) >= 0;

      if (!gameEnabled || !absoluteEnabled)
      {
        if (gameEnabled)
          ASensorEventQueue_disableSensor(queue, gameSensor);
        if (absoluteEnabled)
          ASensorEventQueue_disableSensor(queue, absoluteSensor);
        if (magneticEnabled)
          ASensorEventQueue_disableSensor(queue, magneticSensor);
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
      if (magneticEnabled)
      {
        ASensorEventQueue_setEventRate(
            queue,
            magneticSensor,
            kUpdatePeriodMicroseconds);
      }
      available.store(true);

      while (running.load())
      {
        const int ident = ALooper_pollOnce(100, nullptr, nullptr, nullptr);
        if (ident != kSensorLooperId)
          continue;

        ASensorEvent event;
        while (ASensorEventQueue_getEvents(queue, &event, 1) > 0)
        {
          std::lock_guard lock(mutex);

          if (event.type == ASENSOR_TYPE_GAME_ROTATION_VECTOR)
          {
            latestGame = {
                eventQuaternion(event),
                static_cast<std::int64_t>(event.timestamp),
            };
            hasGame = true;
            updateAlignmentLocked();
          }
          else if (event.type == ASENSOR_TYPE_ROTATION_VECTOR)
          {
            latestAbsolute = {
                eventQuaternion(event),
                static_cast<std::int64_t>(event.timestamp),
            };
            hasAbsolute = true;
            updateAlignmentLocked();
          }
          else if (
              event.type == ASENSOR_TYPE_MAGNETIC_FIELD &&
              magneticEnabled)
          {
            magneticFieldMicrotesla = std::hypot(
                static_cast<double>(event.data[0]),
                static_cast<double>(event.data[1]),
                static_cast<double>(event.data[2]));
          }
        }
      }

      available.store(false);
      ASensorEventQueue_disableSensor(queue, gameSensor);
      ASensorEventQueue_disableSensor(queue, absoluteSensor);
      if (magneticEnabled)
        ASensorEventQueue_disableSensor(queue, magneticSensor);
      ASensorManager_destroyEventQueue(manager, queue);
    }
  };

  GameRotationSensor::GameRotationSensor() : impl_(std::make_unique<Impl>()) {}
  GameRotationSensor::~GameRotationSensor() = default;

  bool GameRotationSensor::latest(Quaternion &orientation) const
  {
    std::lock_guard lock(impl_->mutex);
    if (!impl_->hasAligned)
      return false;
    orientation = impl_->latestAligned;
    return true;
  }

  bool GameRotationSensor::isAvailable() const
  {
    return impl_->available.load();
  }

} // namespace cumquat::android

#else

namespace cumquat::android
{

  struct GameRotationSensor::Impl
  {
  };

  GameRotationSensor::GameRotationSensor() : impl_(std::make_unique<Impl>()) {}
  GameRotationSensor::~GameRotationSensor() = default;

  bool GameRotationSensor::latest(Quaternion &) const
  {
    return false;
  }

  bool GameRotationSensor::isAvailable() const
  {
    return false;
  }

} // namespace cumquat::android

#endif
