package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.function.DoubleSupplier;

public class Sub1W2W3 {
  public static final String NAME = "Sub1W2W3";

  private static final PathPlannerPath SUB1_W2 = PathPlannerPath.fromPathFile("Sub1W2");

  /**
   * Retrieves a composite Command that represents the full autonomous routine.
   *
   * @param drivetrain The DriveSubsystem required for path following commands.
   * @param objectDetectionSubsystem The ObjectDetectionSubsystem required for note detection.
   * @param manipulator The ManipulatorSubsystem required for shooting and note handling.
   * @return The entire autonomous routine command.
   */
  public static Command get(
      CommandSwerveDrivetrain drivetrain,
      ManipulatorSubsystem manipulator,
      DoubleSupplier distanceToSpeaker) {
    return Commands.sequence(
        manipulator.distanceShoot(distanceToSpeaker),
        Commands.parallel(AutoBuilder.followPath(SUB1_W2), manipulator.intakeNote()),
        manipulator.distanceShoot(distanceToSpeaker));
  }
}
