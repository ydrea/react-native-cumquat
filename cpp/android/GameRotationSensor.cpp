#include "GameRotationSensor.h"
#include "RotationAlignment.h"

#ifdef __ANDROID__

#include <android/looper.h>
#include <android/sensor.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <thread>

namespace cumquat::android
{
  namespace
  {
    constexpr int kSensorLooperId = 1;
    constexpr int kUpdatePeriodMicroseconds = 16667;
  }

  struct GameRotationSensor::Impl
  {
    std::atomic<bool> running{true};
    std::atomic<bool> available{false};
    mutable std::mutex mutex;
    Quaternion latestGame;
    bool hasGame{false};
    std::thread worker;

    Impl() : worker([this]
                    { run(); }) {}

    ~Impl()
    {
      running.store(false);
      if (worker.joinable())
        worker.join();
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
      if (!gameSensor)
        return;

      ASensorEventQueue *queue = ASensorManager_createEventQueue(
          manager,
          looper,
          kSensorLooperId,
          nullptr,
          nullptr);
      if (!queue)
        return;

      if (ASensorEventQueue_enableSensor(queue, gameSensor) < 0)
      {
        ASensorManager_destroyEventQueue(manager, queue);
        return;
      }

      ASensorEventQueue_setEventRate(
          queue,
          gameSensor,
          kUpdatePeriodMicroseconds);
      available.store(true);

      while (running.load())
      {
        const int ident = ALooper_pollOnce(100, nullptr, nullptr, nullptr);
        if (ident != kSensorLooperId)
          continue;

        ASensorEvent event;
        while (ASensorEventQueue_getEvents(queue, &event, 1) > 0)
        {
          if (event.type != ASENSOR_TYPE_GAME_ROTATION_VECTOR)
            continue;

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

          std::lock_guard lock(mutex);
          latestGame = orientation;
          hasGame = true;
        }
      }

      available.store(false);
      ASensorEventQueue_disableSensor(queue, gameSensor);
      ASensorManager_destroyEventQueue(manager, queue);
    }
  };

  GameRotationSensor::GameRotationSensor() : impl_(std::make_unique<Impl>()) {}
  GameRotationSensor::~GameRotationSensor() = default;

  bool GameRotationSensor::latest(Quaternion &orientation) const
  {
    std::lock_guard lock(impl_->mutex);
    if (!impl_->hasGame)
      return false;
    orientation = impl_->latestGame;
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
