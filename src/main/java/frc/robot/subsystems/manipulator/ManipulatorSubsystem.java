package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.manipulator.ShooterConstants;
import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;

public class ManipulatorSubsystem extends SubsystemBase {
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;

  /**
   * Constructor for the ManipulatorSubsystem class. Initializes intake, and shooter/aimer
   * subsystems.
   */
  public ManipulatorSubsystem() {
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
  }

  public Command intakeGrabCommand() {
    return Commands.either(
        intake.runOnce(() -> intake.setIntake(IntakeSpeed.HALF)),
        intake.runOnce(intake::stop),
        intake::isNoteInRoller);
  }

  /*
   * homes the shooter
   */

  public Command shooterHomeCommand() {
    return Commands.sequence(
        Commands.run(() -> shooter.setAngleSpeed(ShooterConstants.AngleMotor.HOME_SPEED), this),
        Commands.waitUntil(shooter::isHome),
        Commands.run(shooter::stopAngleMotor, this));
  }

  public Command shooterToPositionCommand(double position) {
    return Commands.sequence(
        Commands.run(() -> shooter.setPosition(position), this),
        Commands.waitUntil(shooter::atTargetPosition));
  }

  public Command setShooterVelocityCommand(double velocity) {
    return Commands.sequence(
        Commands.run(() -> shooter.setShooter(velocity), this),
        Commands.waitUntil(shooter::atTargetVelocity));
  }
}
