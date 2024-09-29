// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.bearbotics.util.ProcessedJoystick;
import frc.bearbotics.util.ProcessedJoystick.JoystickAxis;
import frc.bearbotics.util.ProcessedJoystick.ThrottleProfile;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import org.photonvision.PhotonUtils;

public class RobotContainer {
  /* Our drivetrain */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final LocalizationSubsystem localization = new LocalizationSubsystem(drivetrain);

  /* Our manipulator */
  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();

  /* Our driver joystick */
  private final CommandXboxController driverJoystick =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  /* Our processed joystick inputs */
  private ThrottleProfile throttleProfile = ThrottleProfile.NORMAL;
  private ProcessedJoystick processedJoystick =
      new ProcessedJoystick(driverJoystick, this::getThrottleProfile);

  private AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private Pose2d redSpeakerCenterPose2d;

  public RobotContainer() {
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    redSpeakerCenterPose2d =
        layout.getTagPose(VisionConstants.TAG.RED_SPEAKER_CENTER.tagNumber).get().toPose2d();

    configureBindings();
    setupShuffleboardTab(RobotConstants.COMPETITION_TAB);
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.add("Home Elevator", manipulatorSubsystem.getElevatorHomeCommand());
    shuffleboardTab.addDouble("Distance to Speaker", this::getDistanceToSpeaker);
  }

  private double getDistanceToSpeaker() {
    return PhotonUtils.getDistanceToPose(drivetrain.getPose(), redSpeakerCenterPose2d);
  }

  /** Configure the joystick bindings for the robot. */
  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::getDefaultDriveRequest));
    drivetrain.registerTelemetry(DriveConstants.LOGGER::telemeterize);

    driverJoystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    driverJoystick
        .leftStick()
        .whileTrue(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.TURBO)))
        .onFalse(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.NORMAL)));

    driverJoystick
        .rightStick()
        .whileTrue(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.TURTLE)))
        .onFalse(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.NORMAL)));
  }

  /**
   * Gets the default drive request using field centric mode..
   *
   * @return The default drive request.
   */
  private SwerveRequest getDefaultDriveRequest() {
    return DriveConstants.FIELD_CENTRIC_SWERVE_REQUEST
        .withVelocityX(processedJoystick.get(JoystickAxis.Ly))
        .withVelocityY(processedJoystick.get(JoystickAxis.Lx))
        .withRotationalRate(processedJoystick.get(JoystickAxis.Rx));
  }

  /**
   * Sets the throttle profile for the robot.
   *
   * @param throttleProfile An enum representing the throttle profile.
   */
  private void setThrottleProfile(ThrottleProfile throttleProfile) {
    this.throttleProfile = throttleProfile;
  }

  /**
   * Get the current throttle profile for the robot.
   *
   * @return The throttle profile
   */
  private ThrottleProfile getThrottleProfile() {
    return throttleProfile;
  }

  /**
   * Get the autonomous command.
   *
   * @return the autonomous command.
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
