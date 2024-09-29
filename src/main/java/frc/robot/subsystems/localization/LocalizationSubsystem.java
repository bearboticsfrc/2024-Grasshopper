package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;

public class LocalizationSubsystem extends SubsystemBase {
  EstimationRunnable estimatorRunnable;
  private CommandSwerveDrivetrain swerve;

  private LocalizingCamera camera =
      new LocalizingCamera(
          "camera", new PhotonCamera(VisionConstants.CAMERA_NAME), VisionConstants.CAMERA_TO_ROBOT);

  private StructPublisher<Pose2d> fusedPosePublisher;
  private DoublePublisher headingPublisher;
  private DoublePublisher distancePublisher;

  public LocalizationSubsystem(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    this.estimatorRunnable = new EstimationRunnable(camera.getNiceName(), camera);

    Notifier notifier = new Notifier(() -> estimatorRunnable.run());
    notifier.setName("AprilTagCameras");
    notifier.startPeriodic(RobotConstants.CYCLE_TIME);

    fusedPosePublisher =
        NetworkTableInstance.getDefault().getStructTopic("/vision/pose", Pose2d.struct).publish();

    headingPublisher =
        NetworkTableInstance.getDefault().getDoubleTopic("/vision/heading").publish();

    tab.addString("Pose", () -> StringFormatting.poseToString(swerve.getPose()));
  }

  /**
   * Checks and processes the latest pose estimate from an EstimationRunnable. If a new estimate is
   * available, it is used to update the robot's pose in the DriveSubsystem.
   */
  public void estimatorChecker() {
    CameraPoseResultantIdentity robotPose = this.estimatorRunnable.getLatestPose();

    if (robotPose == null) {
      return;
    }
    Pose2d visionPose = robotPose.getPose2d(VisionConstants.CAMERA_TO_ROBOT);

    swerve.addVisionMeasurement(visionPose, robotPose.getTimestampSeconds());
  }

  @Override
  public void periodic() {
    estimatorChecker();
    fusedPosePublisher.set(swerve.getPose());
    headingPublisher.set(
        swerve.getPose().getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees());
  }
}
