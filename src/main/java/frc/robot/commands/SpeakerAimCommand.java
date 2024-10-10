package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.bearbotics.location.FieldPositions;
import frc.robot.constants.commands.SpeakerAimConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonUtils;

public class SpeakerAimCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private final PIDController rotationalPIDController =
      new PIDController(
          SpeakerAimConstants.RotationPid.P,
          SpeakerAimConstants.RotationPid.I,
          SpeakerAimConstants.RotationPid.D);

  private final Debouncer rotationSetpointDebouncer = new Debouncer(0.1);

  private final SwerveRequest.FieldCentric swerveRequest =
      new SwerveRequest.FieldCentric().withDeadband(0.1);
  private final SwerveRequest.Idle idleSwerveRequest = new SwerveRequest.Idle();

  private DoubleSupplier xVelocitySupplier = () -> 0;
  private DoubleSupplier yVelocitySupplier = () -> 0;

  private Pose2d speakerPose;
  private boolean endless = false;

  /**
   * Constructs the SpeakerAimCommand for aiming at the speaker with x and y velocity suppliers for
   * dynanic driving while aiming
   *
   * @param drivetrain The drivetrain instance
   * @param xVelocitySupplier The x velocity supplier
   * @param yVelocitySupplier The y velocity supplier
   */
  public SpeakerAimCommand(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier) {
    this(drivetrain);

    this.endless = true;
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
  }

  /**
   * Constructs the SpeakerAimCommand for aiming at the speaker
   *
   * @param drivetrain The drivetrain instance for robot movement control.
   */
  public SpeakerAimCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    rotationalPIDController.enableContinuousInput(
        SpeakerAimConstants.RotationPid.ContinuousInput.MIN,
        SpeakerAimConstants.RotationPid.ContinuousInput.MAX);

    rotationalPIDController.setTolerance(SpeakerAimConstants.SETPOINT_TOLERANCE.getRadians());
    rotationalPIDController.setSetpoint(Math.toRadians(180));

    addRequirements(drivetrain);
  }

  /** Initalize, set the speaker pose, stop the robot */
  @Override
  public void initialize() {
    speakerPose = FieldPositions.getInstance().getSpeakerCenter();

    drivetrain.setControl(idleSwerveRequest);
  }

  /** Align the robot's heading with the speaker's pose */
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
    double rotateOutput = -rotationalPIDController.calculate(offsetRotation.getRadians());

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
    return endless
        ? false
        : rotationSetpointDebouncer.calculate(rotationalPIDController.atSetpoint());
  }

  /** Stops any robot movement */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(idleSwerveRequest);
  }
}
