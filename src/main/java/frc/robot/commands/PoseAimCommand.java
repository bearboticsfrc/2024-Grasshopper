package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.commands.PoseAimConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonUtils;

public class PoseAimCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private final PIDController rotationalPIDController =
      new PIDController(
          PoseAimConstants.RotationPid.P,
          PoseAimConstants.RotationPid.I,
          PoseAimConstants.RotationPid.D);

  private final Debouncer rotationSetpointDebouncer = new Debouncer(0.1);

  private final SwerveRequest.FieldCentric swerveRequest =
      new SwerveRequest.FieldCentric().withDeadband(0.1);
  private final SwerveRequest.Idle idleSwerveRequest = new SwerveRequest.Idle();

  private DoubleSupplier xVelocitySupplier = () -> 0;
  private DoubleSupplier yVelocitySupplier = () -> 0;

  private Supplier<Pose2d> targetPose;
  private boolean endless = false;

  /**
   * Constructs the SpeakerAimCommand for aiming at the speaker with x and y velocity suppliers for
   * dynanic driving while aiming
   *
   * @param drivetrain The drivetrain instance
   * @param xVelocitySupplier The x velocity supplier
   * @param yVelocitySupplier The y velocity supplier
   */
  public PoseAimCommand(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> targetPose,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier) {
    this(drivetrain, targetPose);

    this.endless = true;
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
  }

  /**
   * Constructs the SpeakerAimCommand for aiming at the speaker
   *
   * @param drivetrain The drivetrain instance for robot movement control.
   */
  public PoseAimCommand(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;

    rotationalPIDController.enableContinuousInput(
        PoseAimConstants.RotationPid.ContinuousInput.MIN,
        PoseAimConstants.RotationPid.ContinuousInput.MAX);

    rotationalPIDController.setTolerance(PoseAimConstants.SETPOINT_TOLERANCE.getRadians());
    rotationalPIDController.setSetpoint(Math.toRadians(180));

    addRequirements(drivetrain);
  }

  /** Initalize, set the pose, stop the robot */
  @Override
  public void initialize() {
    drivetrain.setControl(idleSwerveRequest);
  }

  /** Align the robot's heading with the pose */
  @Override
  public void execute() {
    drivetrain.setControl(getSwerveRequest());
  }

  /**
   * Get the swerve request to align with the pose
   *
   * @return A swerve request describing how we want to align
   */
  public SwerveRequest getSwerveRequest() {
    Rotation2d offsetRotation = getYawToPose();
    double rotateOutput = -rotationalPIDController.calculate(offsetRotation.getRadians());

    return swerveRequest
        .withVelocityX(xVelocitySupplier.getAsDouble())
        .withVelocityY(yVelocitySupplier.getAsDouble())
        .withRotationalRate(rotateOutput);
  }

  /**
   * Get the yaw offset to the pose
   *
   * @return The yaw
   */
  private Rotation2d getYawToPose() {
    return PhotonUtils.getYawToPose(drivetrain.getPose(), targetPose.get());
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
