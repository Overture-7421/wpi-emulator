#include <doctest/doctest.h>

#include <limits>
#include <vector>

#include "wpi/hal/CAN.h"
#include "wpi/hal/IMU.h"
#include "wpi/hal/PWM.h"
#include "wpi/hal/SimDevice.h"
#include "wpi/hal/simulation/AddressableLEDData.h"
#include "wpi/hal/simulation/EncoderData.h"
#include "wpiemu/bridges/hal/hal_sim_binding.h"

using namespace wpiemu;

TEST_CASE("2027 HAL: LED banks read their pixel range without inventing port or running state") {
    auto api = bridges::hal::makeHalSimApi();
    HALSIM_SetAddressableLEDInitialized(0, true);
    HALSIM_SetAddressableLEDStart(0, 7);
    HALSIM_SetAddressableLEDLength(0, 2);
    HAL_AddressableLEDData data[2]{};
    data[0].r = 12;
    data[1].b = 56;
    HALSIM_SetAddressableLEDData(7, 2, data);
    std::vector<bridges::hal::LedPixel> pixels;
    api->addressableLed().readPixels(0, pixels);
    REQUIRE(pixels.size() == 2);
    CHECK(pixels[0].r == 12);
    CHECK(pixels[1].b == 56);
    HALSIM_ResetAddressableLEDData(0);
}

TEST_CASE("2027 HAL: capabilities, PWM throttle, encoder rate and IMU preserve units") {
    auto api = bridges::hal::makeHalSimApi();
    CHECK(api->analogOut() == nullptr);
    CHECK(api->relay() == nullptr);
    CHECK(api->accelerometer() == nullptr);
    CHECK_FALSE(api->roboRio().hasInputCurrent());
    CHECK_FALSE(api->addressableLed().hasOutputPort());
    CHECK_FALSE(api->addressableLed().hasRunning());
    std::int32_t status = 0;
    const auto pwm = HAL_InitializePWMPort(0, "2027 test", &status);
    REQUIRE(status == 0);
    const auto device = HAL_CreateSimDevice("PWMMotorController[0]");
    const auto throttle = HAL_CreateSimValueDouble(device, "Throttle", HAL_SIM_VALUE_OUTPUT, -0.75);
    HAL_SetPWMSimDevice(pwm, device);
    CHECK(api->pwm().hasSpeed(0));
    CHECK(api->pwm().speed(0) == doctest::Approx(-0.75));
    HAL_SetSimValueDouble(throttle, 0.25);
    CHECK(api->pwm().speed(0) == doctest::Approx(0.25));
    HAL_FreePWMPort(pwm);
    HAL_FreeSimDevice(device);

    HALSIM_SetEncoderDistancePerPulse(0, 0.02);
    api->encoder().setPeriod(0, -0.01);
    CHECK(HALSIM_GetEncoderRate(0) == doctest::Approx(-2.0));
    api->encoder().setPeriod(0, std::numeric_limits<double>::infinity());
    CHECK(HALSIM_GetEncoderRate(0) == 0.0);
    REQUIRE(api->imu() != nullptr);
    api->imu()->setAcceleration(1.0, 2.0, 9.81);
    api->imu()->setAngles(0.1, 0.2, 0.3);
    api->imu()->setRates(0.4, 0.5, 0.6);
    HAL_Acceleration3d accel{};
    HAL_GetIMUAcceleration(&accel, &status);
    CHECK(accel.z == doctest::Approx(9.81));
    HAL_GyroRate3d rate{};
    HAL_GetIMUGyroRates(&rate, &status);
    CHECK(rate.z == doctest::Approx(0.6));
    CHECK(HAL_GetIMUYawFlat(nullptr) == doctest::Approx(0.3));
}

TEST_CASE("2027 HAL: CAN-FD callback preserves the observed bus and final payload byte") {
    struct Listener final : bridges::hal::ICanFrameListener {
        void onFrame(const bridges::hal::CanFrame& value) override { frames.push_back(value); }
        std::vector<bridges::hal::CanFrame> frames;
    } listener;
    auto api = bridges::hal::makeHalSimApi();
    auto subscription = api->can().subscribe(listener);
    REQUIRE(subscription);
    HAL_CANMessage message{};
    message.dataSize = 64;
    for (int i = 0; i < 64; ++i) message.data[i] = static_cast<std::uint8_t>(i);
    std::int32_t status = 0;
    HAL_CAN_SendMessage(2, 0x0205000c, &message, 20, &status);
    REQUIRE(listener.frames.size() == 1);
    CHECK(listener.frames[0].busId == 2);
    CHECK(listener.frames[0].arbitrationId == 0x0205000c);
    CHECK(listener.frames[0].periodMs == 20);
    CHECK(listener.frames[0].size == 64);
    CHECK(listener.frames[0].data[63] == 63);
}
