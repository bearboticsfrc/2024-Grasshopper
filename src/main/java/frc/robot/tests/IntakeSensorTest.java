package frc.robot.tests;

import edu.wpi.first.wpilibj.Timer;
import frc.bearbotics.tests.Testable;
import frc.bearbotics.tests.TestableInterface;
import frc.robot.subsystems.manipulator.IntakeSubsystem;

public class IntakeSensorTest extends Testable implements TestableInterface {
  private final double WAIT_SECONDS = 3;

  private IntakeSubsystem intake;

  public IntakeSensorTest(IntakeSubsystem intake) {
    this.intake = intake;
  }

  @Override
  public void run() throws AssertionError {
    Timer.delay(WAIT_SECONDS);

    test(intake.isNoteInRoller(), "Intake roller sensor not detecting a breakage");
    test(intake.isNoteInShooter(), "Intake shooter sensor not detecting a breakage");
  }

  @Override
  public void end() {}
}
