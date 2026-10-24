#include <iostream>

#include <wpi/drivers/motor/PWMSparkMax.hpp>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/framework/TimedRobot.hpp>
#include <wpi/hardware/discrete/DigitalInput.hpp>

class Robot final : public wpi::TimedRobot {
public:
    void TeleopInit() override { std::cout << "SEASON_SAMPLE teleop" << std::endl; }
    void TeleopPeriodic() override { motor_.SetThrottle(limit_.Get() ? 0.25 : 0.5); }
    void DisabledInit() override { motor_.StopMotor(); }

private:
    wpi::PWMSparkMax motor_{0};
    wpi::DigitalInput limit_{1};
};

int main() { return wpi::StartRobot<Robot>(); }
