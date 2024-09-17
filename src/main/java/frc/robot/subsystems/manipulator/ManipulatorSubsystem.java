package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.manipulator.ElevatorConstants;
import frc.robot.constants.manipulator.ElevatorConstants.ElevatorPosition;
import frc.robot.constants.manipulator.IntakeConstants.IntakeVelocity;
import frc.robot.constants.manipulator.ShooterConstants.ShooterVelocity;

public class ManipulatorSubsystem extends SubsystemBase {
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final ElevatorSubsystem angle;

  /**
   * Constructor for the ManipulatorSubsystem class. Initializes intake, and shooter/aimer
   * subsystems.
   */
  public ManipulatorSubsystem() {
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    angle = new ElevatorSubsystem();
  }

  /**
   * Get the angle subsytem home command
   *
   * @return The command
   */
  public Command getAngleHomeCommand() {
    return Commands.sequence(
        angle.runOnce(() -> angle.set(ElevatorConstants.HOMING_SPEED)),
        Commands.waitUntil(angle::isAtLowerLimit),
        angle.runOnce(angle::stopMotor));
  }

  /**
   * Get the intake command
   *
   * @return The command
   */
  public Command getIntakeCommand() {
    return Commands.either(
        Commands.idle(),
        Commands.sequence(
            intake.runOnce(() -> intake.set(IntakeVelocity.HALF)),
            Commands.waitUntil(intake::isNoteInRoller),
            intake.runOnce(intake::stop)),
        intake::isNoteInRoller);
  }

  /**
   * Get the intake set command
   *
   * @param velocity The velocity to set the intake motors to
   * @return The command
   */
  public Command getIntakeSetCommand(IntakeVelocity velocity) {
    return intake.runOnce(() -> intake.set(velocity));
  }

  /**
   * Get the angle subsytem set command
   *
   * @param position The position to set the angle to
   * @return The command
   */
  public Command getAngleSetCommand(ElevatorPosition position) {
    return Commands.sequence(
        angle.runOnce(() -> angle.set(position)), Commands.waitUntil(angle::atTargetPosition));
  }

  /**
   * Get the shooter subsytem set command
   *
   * @param velocity The velocity to set the shooter to
   * @return The command
   */
  public Command getShooterSetCommand(ShooterVelocity velocity) {
    return Commands.sequence(
        shooter.runOnce(() -> shooter.set(velocity)),
        Commands.waitUntil(shooter::atTargetVelocity));
  }
}
