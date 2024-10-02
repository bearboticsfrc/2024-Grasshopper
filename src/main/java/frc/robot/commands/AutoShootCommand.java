package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.bearbotics.location.FieldPositions;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.function.Supplier;
import org.photonvision.PhotonUtils;

public class AutoShootCommand extends SequentialCommandGroup {
  private final CommandSwerveDrivetrain drivetrain;
  private final ManipulatorSubsystem manipulatorSubsystem;

  private final Supplier<Pose2d> targetPoseSupplier;

  /**
   * Constructs the AutoShootCommand with the DriveSubsystem and ManipulatorSubsystem.
   *
   * @param driveSubsystem The DriveSubsystem instance.
   * @param manipulatorSubsystem The ManipulatorSubsystem instance control.
   */
  public AutoShootCommand(
      CommandSwerveDrivetrain drivetrain, ManipulatorSubsystem manipulatorSubsystem) {
    this.drivetrain = drivetrain;
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.targetPoseSupplier = FieldPositions.getInstance()::getSpeakerCenter;

    addCommands(getSpeakerAimCommand(), getShootCommand());
    addRequirements(drivetrain, manipulatorSubsystem);
  }

  private Command getSpeakerAimCommand() {
    return new SpeakerAimCommand(drivetrain);
  }

  private Command getShootCommand() {
    return Commands.deferredProxy(
        () ->
            manipulatorSubsystem.distanceShoot(
                () ->
                    PhotonUtils.getDistanceToPose(drivetrain.getPose(), targetPoseSupplier.get())));
  }
}
