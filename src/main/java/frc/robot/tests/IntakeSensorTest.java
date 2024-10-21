package frc.robot.tests;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.manipulator.IntakeSubsystem;

public class IntakeSensorTest implements TestableInterface {
  private final double WAIT_SECONDS = 3;

  private IntakeSubsystem intake;

  public IntakeSensorTest(IntakeSubsystem intake) {
    this.intake = intake;
  }

  public static IntakeSensorTest get(IntakeSubsystem intake) {
    return new IntakeSensorTest(intake);
  }

  @Override
  public void run() throws AssertionError {
    Timer.delay(WAIT_SECONDS);

    assert intake.isNoteInRoller() : "Intake roller sensor not detecting a breakage";
    assert intake.isNoteInShooter() : "Intake shooter sensor not detecting a breakage";
  }
}
