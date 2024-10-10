package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.bearbotics.fms.AllianceColor;
import frc.bearbotics.fms.AllianceReadyListener;
import frc.robot.Vision;
import frc.robot.constants.AutoConstants;
import frc.robot.generated.TunerConstants;
import java.util.Arrays;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;

/**
 * A command-based SwerveDrivetrain class that extends the Phoenix SwerveDrivetrain class and
 * implements WPILib's Subsystem and AllianceReadyListener interfaces. It integrates vision
 * processing and alliance-specific behavior.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain
    implements Subsystem, AllianceReadyListener {
  /* Vision class for pose estimation */
  private final Vision vision = new Vision();

  /* Field-oriented rotation offsets for the respective alliance.  */
  private final Rotation2d blueAllianceRotation = Rotation2d.fromDegrees(0);
  private final Rotation2d redAllianceRotation = Rotation2d.fromDegrees(180);

  /* Swerve request for pathplanner */
  private final SwerveRequest.ApplyChassisSpeeds autoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  /**
   * Constructs a new CommandSwerveDrivetrain object.
   *
   * @param driveTrainConstants Constants for configuring the swerve drivetrain.
   * @param modules The individual swerve module constants for the drivetrain.
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    configurePathPlanner();
    AllianceColor.addListener(this);
  }

  /**
   * Creates a command that applies a swerve request using a supplier.
   *
   * @param requestSupplier The supplier that provides the SwerveRequest.
   * @return A command that applies the given swerve request.
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> setControl(requestSupplier.get()));
  }

  /**
   * Uses vision data to update the drivetrain's pose estimate and applies vision measurements to
   * the pose estimator.
   */
  @Override
  public void periodic() {
    vision.getEstimatedGlobalPose().ifPresent(this::addVisionMeasurement);
  }

  /**
   * Adds vision-based pose estimation measurements to the drivetrain.
   *
   * @param estimatedRobotPose The estimated robot pose from vision processing.
   */
  public void addVisionMeasurement(EstimatedRobotPose estimatedRobotPose) {
    Pose2d estPose = estimatedRobotPose.estimatedPose.toPose2d();
    Matrix<N3, N1> estStdDevs = vision.getEstimationStdDevs(estPose);
    addVisionMeasurement(estPose, estimatedRobotPose.timestampSeconds, estStdDevs);
  }

  /** Configures PathPlanner. */
  private void configurePathPlanner() {
    double driveBaseRadius =
        Arrays.stream(m_moduleLocations)
            .mapToDouble(moduleLocation -> moduleLocation.getNorm())
            .max()
            .orElse(0);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::seedFieldRelative,
        this::getCurrentRobotChassisSpeeds,
        this::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            AutoConstants.HolonomicPathFollower.TRANSLATION_PID_CONSTANTS,
            AutoConstants.HolonomicPathFollower.ROTATIONAL_PID_CONSTANTS,
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () -> AllianceColor.getAlliance() == Alliance.Red,
        this);
  }

  /**
   * Retrieves the current chassis speeds of the robot by converting the module states into chassis
   * speeds.
   *
   * @return The current chassis speeds of the robot.
   */
  private ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  /**
   * Sets the chassis speeds for the robot based on the given ChassisSpeeds.
   *
   * @param chassisSpeeds The chassis speeds to apply to the robot.
   */
  private void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setControl(autoRequest.withSpeeds(chassisSpeeds));
  }

  /**
   * Retrieves the current pose of the robot.
   *
   * @return The current Pose2d of the robot.
   */
  public Pose2d getPose() {
    return getState().Pose;
  }

  /**
   * Updates the field-centric driving perspective of the operator based on the robot's current
   * alliance. The perspective is set to face forward relative to the alliance's side of the field.
   *
   * @param alliance The current alliance (Red or Blue).
   */
  @Override
  public void updateAlliance(Alliance alliance) {
    setOperatorPerspectiveForward(
        alliance == Alliance.Red ? redAllianceRotation : blueAllianceRotation);
  }
}
