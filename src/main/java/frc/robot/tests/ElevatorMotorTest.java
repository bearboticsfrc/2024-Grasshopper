package frc.robot.tests;

import frc.bearbotics.tests.Testable;
import frc.bearbotics.tests.TestableInterface;
import frc.robot.constants.manipulator.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;

public class ElevatorMotorTest extends Testable implements TestableInterface {
  private final double WAIT_SECONDS = 1;
  private final double[] TEST_POSITIONS = {5, 10, 20, 30};

  private ElevatorSubsystem elevator;

  public ElevatorMotorTest(ElevatorSubsystem elevator) {
    this.elevator = elevator;
  }

  @Override
  public void run() throws AssertionError {
    for (double position : TEST_POSITIONS) {
      wait(() -> elevator.setPosition(position), WAIT_SECONDS);

      test(
          elevator.atTargetPosition(),
          "Elevator not reaching target position (%g)".formatted(position));
    }
  }

  public void end() {
    elevator.setPosition(ElevatorPosition.HOME);
  }
}
