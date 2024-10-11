package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.bearbotics.location.FieldPositions;
import frc.robot.commands.PoseAimCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.function.DoubleSupplier;

public class AutoUtils {
  /**
   * Creates the distance shoot command sequence.
   *
   * @param drivetrain The swerve drivetrain subsystem.
   * @param manipulator The manipulator subsystem.
   * @param distanceToSpeaker The supplier for distance to the speaker.
   * @return The command sequence for aiming and shooting.
   */
  public static Command distanceShoot(
      CommandSwerveDrivetrain drivetrain,
      ManipulatorSubsystem manipulator,
      DoubleSupplier distanceToSpeaker) {
    return Commands.sequence(
        new PoseAimCommand(drivetrain, FieldPositions.getInstance()::getSpeakerCenter),
        manipulator.distanceShoot(distanceToSpeaker));
  }

  /**
   * Executes the intake action while following the specified path.
   *
   * <p>This method creates a parallel command that allows the robot to perform the intake operation
   * concurrently with navigating along a defined path.
   *
   * @param path The path to follow during the autonomous routine.
   * @param manipulator The manipulator subsystem responsible for the intake action.
   * @return A Command that executes the intake while following the path.
   */
  public static Command followPathAndIntake(
      PathPlannerPath path, ManipulatorSubsystem manipulator) {
    return Commands.parallel(AutoBuilder.followPath(path), manipulator.intakeNote());
  }
}
