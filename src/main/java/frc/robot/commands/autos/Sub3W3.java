package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.function.DoubleSupplier;

public class Sub3W3 implements AutoInterface {
  private final PathPlannerPath SUB3_W3 = PathPlannerPath.fromPathFile("Sub3W3");

  /**
   * Retrieve the name of this auto
   *
   * @return The auto name
   */
  public String getName() {
    return "Sub3W3";
  }

  /**
   * Retrieves a composite Command that represents the full autonomous routine.
   *
   * @param drivetrain The DriveSubsystem required for path following commands.
   * @param objectDetectionSubsystem The ObjectDetectionSubsystem required for note detection.
   * @param manipulator The ManipulatorSubsystem required for shooting and note handling.
   * @return The entire autonomous routine command.
   */
  public Command get(
      CommandSwerveDrivetrain drivetrain,
      ManipulatorSubsystem manipulator,
      DoubleSupplier distanceToSpeaker) {
    return Commands.sequence(
        AutoUtils.distanceShoot(drivetrain, manipulator, distanceToSpeaker),
        AutoUtils.followPathAndIntake(SUB3_W3, manipulator),
        AutoUtils.distanceShoot(drivetrain, manipulator, distanceToSpeaker));
  }
}
