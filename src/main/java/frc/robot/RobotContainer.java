// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.bearbotics.location.FieldPositions;
import frc.bearbotics.util.ProcessedJoystick;
import frc.bearbotics.util.ProcessedJoystick.JoystickAxis;
import frc.bearbotics.util.ProcessedJoystick.ThrottleProfile;
import frc.robot.commands.SpeakerAimCommand;
import frc.robot.commands.autos.Sub1W2W3;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import org.photonvision.PhotonUtils;

public class RobotContainer {
  /* Our drivetrain */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  /* Our manipulator */
  private final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();

  /* Our driver joystick */
  private final CommandXboxController driverJoystick =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  /* Our processed joystick inputs */
  private final ProcessedJoystick processedJoystick =
      new ProcessedJoystick(driverJoystick, this::getThrottleProfile);
  private ThrottleProfile throttleProfile = ThrottleProfile.NORMAL;

  /* Our auto command choose on Shuffleboard */
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    setupShuffleboardTab(RobotConstants.COMPETITION_TAB);
    buildAutoList();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.add("Home Elevator", manipulator.homeElevator());
    shuffleboardTab.addDouble("Distance to Speaker", this::getDistanceToSpeaker);
  }

  /** Configure the joystick bindings for the robot. */
  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::getDefaultDriveRequest));
    drivetrain.registerTelemetry(DriveConstants.LOGGER::telemeterize);

    driverJoystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    driverJoystick
        .rightStick()
        .whileTrue(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.TURBO)))
        .onFalse(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.NORMAL)));

    driverJoystick
        .leftStick()
        .whileTrue(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.TURTLE)))
        .onFalse(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.NORMAL)));

    driverJoystick
        .leftTrigger()
        .whileTrue(manipulator.intakeNote())
        .onFalse(manipulator.stopIntake());

    driverJoystick
        .rightTrigger()
        .whileTrue(
            new SpeakerAimCommand(
                drivetrain,
                () -> processedJoystick.get(JoystickAxis.Ly),
                () -> processedJoystick.get(JoystickAxis.Lx)));

    driverJoystick
        .leftBumper()
        .whileTrue(manipulator.subwooferShoot())
        .onFalse(manipulator.stopManipulator());

    driverJoystick
        .rightBumper()
        .whileTrue(manipulator.distanceShoot(this::getDistanceToSpeaker))
        .onFalse(manipulator.stopManipulator());
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
   * Get the distance (in meters) to the speaker center (tag 4 for red alliance)
   *
   * @return The distance, in meters
   */
  private double getDistanceToSpeaker() {
    return PhotonUtils.getDistanceToPose(
        drivetrain.getPose(), FieldPositions.getInstance().getSpeakerCenter());
  }
  /**
   * Provides an override for the robot's rotation target during autonomous path following. This can
   * be used to dynamically adjust the robot's orientation based on strategic needs.
   *
   * @return An Optional containing the new rotation target, or an empty Optional if no override is
   *     needed.
   */
  private void buildAutoList() {
    FollowPathCommand.warmupCommand().schedule();

    autoCommandChooser.setDefaultOption("0 - NoOp", Commands.idle());
    autoCommandChooser.addOption(
        "1 - Sub1W2W3", Sub1W2W3.get(drivetrain, manipulator, this::getDistanceToSpeaker));

    RobotConstants.COMPETITION_TAB
        .add("Auto Command", autoCommandChooser)
        .withSize(4, 1)
        .withPosition(0, 1);
  }

  /**
   * Get the autonomous command.
   *
   * @return the autonomous command.
   */
  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
  }

  /** Ran on teleop init. Stops the manipulator */
  public void teleopInit() {
    manipulator.stopManipulator().schedule();
  }
}
