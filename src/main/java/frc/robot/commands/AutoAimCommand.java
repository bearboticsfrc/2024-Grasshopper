package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.commands.AutoAimConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.*;
import org.photonvision.PhotonUtils;

public class AutoAimCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private PIDController rotSpeedPidController =
      new PIDController(AutoAimConstants.RotationPid.P, AutoAimConstants.RotationPid.I, 0);

  private final Supplier<Pose2d> targetPoseSupplier;

  /**
   * Constructs the AutoAimCommand with the drivetrain and target point.
   *
   * @param drivetrain The drivetrain instance for robot movement control.
   * @param targetPoseSupplier The target point for auto-aiming.
   */
  public AutoAimCommand(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
    this.drivetrain = drivetrain;
    this.targetPoseSupplier = targetPoseSupplier;

    rotSpeedPidController.enableContinuousInput(
        AutoAimConstants.RotationPid.ContinuousInput.MIN,
        AutoAimConstants.RotationPid.ContinuousInput.MAX);

    rotSpeedPidController.setTolerance(0.07);
    rotSpeedPidController.setSetpoint(0);

    addRequirements(drivetrain);
  }

  /**
   * Executes the auto-aiming logic by aligning the robot's heading with the specified target point.
   */
  @Override
  public void execute() {
    drivetrain.setControl(getSwerveRequest(targetPoseSupplier.get()));
  }

  public SwerveRequest getSwerveRequest(Pose2d targetPose) {
    Rotation2d offsetRotation = PhotonUtils.getYawToPose(drivetrain.getPose(), targetPose);
    double rotateOutput = rotSpeedPidController.calculate(offsetRotation.getRadians());

    SwerveRequest swerveRequest =
        DriveConstants.FIELD_CENTRIC_AUTO_SWERVE_REQUEST.withRotationalRate(rotateOutput);

    return swerveRequest;
  }

  /** Returns true if the PID controller indicates we are aimed. */
  @Override
  public boolean isFinished() {
    return rotSpeedPidController.atSetpoint();
  }
}
