package frc.robot.tests;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.manipulator.IntakeConstants.IntakeVelocity;
import frc.robot.subsystems.manipulator.IntakeSubsystem;

public class IntakeMotorTest implements TestableInterface {
  private final double WAIT_SECONDS = 0.5;

  private IntakeSubsystem intake;

  public IntakeMotorTest(IntakeSubsystem intake) {
    this.intake = intake;
  }

  public static IntakeSensorTest get(IntakeSubsystem intake) {
    return new IntakeSensorTest(intake);
  }

  @Override
  public void run() throws AssertionError {
    intake.setVelocity(IntakeVelocity.FULL);

    Timer.delay(WAIT_SECONDS);
    DataLogManager.log("foo 1");

    assert MathUtil.isNear(1, intake.getVelocity(), 0.05) : "Intake motor not reaching velocity";

    intake.setVelocity(IntakeVelocity.REVERSE);

    Timer.delay(WAIT_SECONDS);

    DataLogManager.log("foo 3");

    assert MathUtil.isNear(-1, intake.getVelocity(), 0.05) : "Intake motor not reaching velocity";

    intake.setVelocity(IntakeVelocity.HALF);

    Timer.delay(WAIT_SECONDS);
    DataLogManager.log("foo 3");

    assert MathUtil.isNear(0.5, intake.getVelocity(), 0.05) : "Intake motor not reaching velocity";
  }
}
