package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.manipulator.ElevatorConstants;
import frc.robot.constants.manipulator.ElevatorConstants.ElevatorPosition;
import frc.robot.constants.manipulator.IntakeConstants;
import frc.robot.constants.manipulator.IntakeConstants.IntakeVelocity;
import frc.robot.constants.manipulator.ShooterConstants.ShooterVelocity;
import java.util.function.DoubleSupplier;

public class ManipulatorSubsystem extends SubsystemBase {
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final ElevatorSubsystem elevator;

  /**
   * Constructor for the ManipulatorSubsystem class. Initializes the intake, shooter, and elevator
   * subsystems.
   */
  public ManipulatorSubsystem() {
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    elevator = new ElevatorSubsystem();
  }

  /**
   * Returns a command to home the elevator subsystem. This command sequence will:
   *
   * <ol>
   *   <li>Max out the elevator encoder
   *   <li>Set the elevator to the homing speed
   *   <li>Wait until the lower limit is reached
   *   <li>Stop the motor
   *   <li>Tare the encoder to reset its position
   * </ol>
   *
   * @return The command to home the elevator.
   */
  public Command homeElevator() {
    return Commands.sequence(
            elevator.runOnce(elevator::maxEncoder),
            elevator.runOnce(() -> elevator.setSpeed(ElevatorConstants.HOMING_SPEED)),
            Commands.waitUntil(elevator::isAtLowerLimit),
            elevator.runOnce(elevator::stopMotor),
            elevator.runOnce(elevator::tareEncoder))
        .withName("Home Elevator");
  }

  /**
   * Returns a command to intake a note. If a note is already in the shooter, the command will idle.
   * Otherwise, it will:
   *
   * <ol>
   *   <li>Set the intake to full velocity
   *   <li>Wait until the note is detected in the shooter
   *   <li>Wait for a brief stop delay
   *   <li>Stop the intake motor
   * </ol>
   *
   * @return The command to intake a note.
   */
  public Command intakeNote() {
    return Commands.either(
        Commands.idle(),
        Commands.sequence(
            intake.runOnce(() -> intake.setVelocity(IntakeVelocity.FULL)),
            Commands.waitUntil(intake::isNoteInShooter),
            Commands.waitSeconds(IntakeConstants.STOP_DELAY),
            intake.runOnce(intake::stopMotor)),
        intake::isNoteInShooter);
  }

  /**
   * Returns a command to feed a note. This command will:
   *
   * <ol>
   *   <li>Set the intake to full velocity
   *   <li>Wait for 0.25 seconds
   *   <li>Stop the manipulator systems (intake, shooter, and elevator)
   * </ol>
   *
   * @return The command to feed a note.
   */
  public Command feedNote() {
    return Commands.sequence(
        setIntake(IntakeVelocity.FULL),
        Commands.waitSeconds(IntakeConstants.FEED_DELAY),
        stopIntake());
  }

  /**
   * Returns a command to set the intake to a specific velocity.
   *
   * @param velocity The velocity to set the intake motors to.
   * @return The command to set the intake.
   */
  public Command setIntake(IntakeVelocity velocity) {
    return intake.runOnce(() -> intake.setVelocity(velocity));
  }

  /**
   * Returns a command to stop the intake.
   *
   * @return The command to stop the intake.
   */
  public Command stopIntake() {
    return intake.runOnce(() -> intake.stopMotor());
  }

  /**
   * Returns a command to stop the shooter.
   *
   * @return The command to stop the shooter.
   */
  public Command stopShooter() {
    return shooter.runOnce(() -> shooter.stopMotor());
  }

  /**
   * Returns a command to stop the elevator.
   *
   * @return The command to stop the elevator.
   */
  public Command stopElevator() {
    return elevator.runOnce(() -> elevator.stopMotor());
  }

  /**
   * Returns a command to stop all manipulator subsystems (intake, shooter, and elevator).
   *
   * @return The command to stop all manipulator subsystems.
   */
  public Command stopManipulator() {
    return Commands.parallel(stopIntake(), stopShooter(), stopElevator());
  }

  /**
   * Returns a command to shoot based on distance. This sequence will:
   *
   * <ol>
   *   <li>Run the shooter and elevator subsystems to set their velocity and position based on
   *       distance
   *   <li>Wait for the shooter and elevator to reach their target
   *   <li>Feed the note
   *   <li>Stop the manipulator
   * </ol>
   *
   * @param distanceSupplier A supplier for the distance to adjust the shooter and elevator.
   * @return The command to shoot based on distance.
   */
  public Command distanceShoot(DoubleSupplier distanceSupplier) {
    return Commands.sequence(
        runShooterAndElevator(distanceSupplier),
        waitForShooterAndElevator(),
        feedNote(),
        stopManipulator());
  }

  /**
   * Returns a command to run the shooter and elevator subsystems based on a supplied distance.
   *
   * @param distanceSupplier A supplier for the distance to adjust the shooter and elevator.
   * @return The command to run the shooter and elevator.
   */
  private Command runShooterAndElevator(DoubleSupplier distanceSupplier) {
    return Commands.parallel(
        shooter.runOnce(() -> shooter.setVelocityFromDistance(distanceSupplier.getAsDouble())),
        elevator.runOnce(() -> elevator.setPositionFromDistance(distanceSupplier.getAsDouble())));
  }

  /**
   * Returns a command that waits for both the shooter and elevator to reach their target velocity
   * and position.
   *
   * @return The command to wait for the shooter and elevator.
   */
  private Command waitForShooterAndElevator() {
    return Commands.parallel(
        Commands.waitUntil(shooter::atTargetVelocity),
        Commands.waitUntil(elevator::atTargetPosition));
  }

  /**
   * Returns a command to set the elevator position.
   *
   * @param position The position to set the elevator to.
   * @return The command to set the elevator.
   */
  public Command setElevator(ElevatorPosition position) {
    return Commands.sequence(
        elevator.runOnce(() -> elevator.setPosition(position)),
        Commands.waitUntil(elevator::atTargetPosition));
  }

  /**
   * Returns a command to set the elevator speed.
   *
   * @param speed The speed to set the elevator to.
   * @return The command to set the elevator.
   */
  public Command setElevator(double speed) {
    return elevator.runOnce(() -> elevator.setSpeed(speed));
  }

  /**
   * Returns a command to set the shooter velocity.
   *
   * @param velocity The velocity to set the shooter to.
   * @return The command to set the shooter.
   */
  public Command setShooter(ShooterVelocity velocity) {
    return Commands.sequence(
        shooter.runOnce(() -> shooter.setVelocity(velocity)),
        Commands.waitUntil(shooter::atTargetVelocity),
        setIntake(IntakeVelocity.FULL));
  }
}
