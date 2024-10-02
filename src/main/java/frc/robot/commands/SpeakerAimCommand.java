package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.bearbotics.location.FieldPositions;
import frc.robot.constants.commands.SpeakerAimConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.*;
import org.photonvision.PhotonUtils;

public class SpeakerAimCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private PIDController rotSpeedPidController =
      new PIDController(SpeakerAimConstants.RotationPid.P, 0, 0);

  private final SwerveRequest.FieldCentric swerveRequest = new FieldCentric();

  private final DoubleSupplier xVelocitySupplier;
  private final DoubleSupplier yVelocitySupplier;

  private Pose2d speakerPose;
  private boolean endless = true;

  /**
   * Constructs the AutoAimCommand with the drivetrain and target point.
   *
   * @param drivetrain The drivetrain instance for robot movement control.
   */
  public SpeakerAimCommand(CommandSwerveDrivetrain drivetrain) {
    this(drivetrain, () -> 0, () -> 0);
    this.endless = false;
  }

  /**
   * Constructs the AutoAimCommand with the drivetrain and target point.
   *
   * @param drivetrain The drivetrain instance for robot movement control.
   */
  public SpeakerAimCommand(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier) {
    this.drivetrain = drivetrain;
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;

    rotSpeedPidController.enableContinuousInput(
        SpeakerAimConstants.RotationPid.ContinuousInput.MIN,
        SpeakerAimConstants.RotationPid.ContinuousInput.MAX);

    rotSpeedPidController.setTolerance(SpeakerAimConstants.SETPOINT_TOLERANCE.getRadians());
    rotSpeedPidController.setSetpoint(0);

    addRequirements(drivetrain);
  }

  /** Initalize, set the speaker pose */
  @Override
  public void initialize() {
    speakerPose = FieldPositions.getInstance().getSpeakerCenter();
  }

  /**
   * Executes the auto-aiming logic by aligning the robot's heading with the specified target point.
   */
  @Override
  public void execute() {
    drivetrain.setControl(getSwerveRequest());
  }

  /**
   * Get the swerve request to align with the speaker
   *
   * @return A swerve request describing how we want to align
   */
  public SwerveRequest getSwerveRequest() {
    Rotation2d offsetRotation = getYawToSpeaker();
    double rotateOutput = rotSpeedPidController.calculate(offsetRotation.getRadians());

    return swerveRequest
        .withVelocityX(xVelocitySupplier.getAsDouble())
        .withVelocityY(yVelocitySupplier.getAsDouble())
        .withRotationalRate(rotateOutput);
  }

  /**
   * Get the yaw offset to the speaker
   *
   * @return The yaw
   */
  private Rotation2d getYawToSpeaker() {
    return PhotonUtils.getYawToPose(drivetrain.getPose(), speakerPose);
  }

  /** Returns true if the PID controller indicates we are aimed. */
  @Override
  public boolean isFinished() {
    return endless ? false : rotSpeedPidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
