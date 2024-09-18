// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.io.File;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class RobotContainer {
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  private SwerveSubsystem swerveSubsystem;
  private ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();

  public RobotContainer() {
    configureSwerveDrive();
    configureBindings();
  }

  /*
   * Configure the swerve drive.
   */
  private void configureSwerveDrive() {
    SwerveDriveTelemetry.verbosity = DriveConstants.TELEMETRY_VERBOSITY;

    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

    try {
      SwerveDrive swerveDrive =
          new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
      swerveSubsystem = new SwerveSubsystem(swerveDrive);
    } catch (IOException exception) {
      DataLogManager.log("IOException - swerveDrive");
      return;
    }
  }

  /*
   * Configure the controller bindings.
   */
  private void configureBindings() {
    setDefaultDriveCommand();
  }

  /*
   * Set the default drive command for this subsystem.
   */
  public void setDefaultDriveCommand() {
    Command driveFieldOrientedDirectAngle =
        swerveSubsystem.driveCommand(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), 0.25),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), 0.25),
            () -> driverController.getRightX());

    swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  /*
   * Get the autonomous command.
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
