package frc.robot.tests;

import frc.bearbotics.tests.Testable;
import frc.bearbotics.tests.TestableInterface;
import frc.robot.subsystems.manipulator.ShooterSubsystem;

public class ShooterMotorTest extends Testable implements TestableInterface {
  private final double WAIT_SECONDS = 1;

  private ShooterSubsystem shooter;

  public ShooterMotorTest(ShooterSubsystem shooter) {
    this.shooter = shooter;
  }

  @Override
  public void run() throws AssertionError {
    wait(() -> shooter.setVelocity(500), WAIT_SECONDS);

    test(shooter.atTargetVelocity(), "Shooter motor not reaching target velocity (500rpm)");

    wait(() -> shooter.setVelocity(1000), WAIT_SECONDS);

    test(shooter.atTargetVelocity(), "Shooter motor not reaching target velocity (1000rpm)");

    wait(() -> shooter.setVelocity(2000), WAIT_SECONDS);

    test(shooter.atTargetVelocity(), "Shooter motor not reaching target velocity (2000rpm)");

    wait(() -> shooter.setVelocity(4000), WAIT_SECONDS);

    test(shooter.atTargetVelocity(), "Shooter motor not reaching target velocity (4000rpm)");
  }

  public void end() {
    shooter.stopMotor();
  }
}
