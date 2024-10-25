package frc.robot.tests;

import edu.wpi.first.math.MathUtil;
import frc.bearbotics.tests.Testable;
import frc.bearbotics.tests.TestableInterface;
import frc.robot.constants.manipulator.IntakeConstants.IntakeVelocity;
import frc.robot.subsystems.manipulator.IntakeSubsystem;

public class IntakeMotorTest extends Testable implements TestableInterface {
  private final double WAIT_SECONDS = 2; // TODO: Refactor to use loop!

  private IntakeSubsystem intake;

  public IntakeMotorTest(IntakeSubsystem intake) {
    this.intake = intake;
  }

  @Override
  public void run() throws AssertionError {
    wait(() -> intake.setVelocity(IntakeVelocity.FULL), WAIT_SECONDS);

    test(MathUtil.isNear(1, intake.getVelocity(), 0.05), "Intake motor not reaching velocity");

    wait(() -> intake.setVelocity(IntakeVelocity.REVERSE), WAIT_SECONDS);

    test(MathUtil.isNear(-1, intake.getVelocity(), 0.05), "Intake motor not reaching velocity");

    wait(() -> intake.setVelocity(IntakeVelocity.HALF), WAIT_SECONDS);

    test(MathUtil.isNear(0.5, intake.getVelocity(), 0.05), "Intake motor not reaching velocity");
  }

  @Override
  public void end() {
    intake.stopMotor();
  }
}
