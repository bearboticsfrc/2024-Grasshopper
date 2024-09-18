// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  /* Our drivetrain */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  /* Our driver joystick */
  private final CommandXboxController driverJoystick =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  /* Our processed joystick inputs */
  private ThrottleProfile throttleProfile = ThrottleProfile.TURBO;
  private ProcessedJoystick processedJoystick =
      new ProcessedJoystick(driverJoystick, this::getThrottleProfile);

  public RobotContainer() {
    configureSwerveDrive();
    configureBindings();
  }

  /** Configure the joystick bindings for the robot. */
  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::getDefaultDriveRequest));

    driverJoystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    driverJoystick
        .leftStick()
        .whileTrue(drivetrain.runOnce(() -> setThrottleProfile(ThrottleProfile.NORMAL)))
        .onFalse(drivetrain.runOnce(() -> setThrottleProfile(ThrottleProfile.TURBO)));

    drivetrain.registerTelemetry(DriveConstants.LOGGER::telemeterize);
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
