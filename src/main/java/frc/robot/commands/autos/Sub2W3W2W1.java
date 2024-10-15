package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.function.DoubleSupplier;

public class Sub2W3W2W1 implements AutoInterface {
  private final PathPlannerPath SUB2_W3 = PathPlannerPath.fromPathFile("Sub2W3");
  private final PathPlannerPath W3_W2 = PathPlannerPath.fromPathFile("W3W2");
  private final PathPlannerPath W2_W1 = PathPlannerPath.fromPathFile("W2W1");

  /**
   * Retrieve the name of this auto
   *
   * @return The auto name
   */
  public String getName() {
    return "Sub2W3W2W1";
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
        manipulator.subwooferShoot(),
        AutoUtils.followPathAndIntake(SUB2_W3, manipulator),
        AutoUtils.distanceShoot(drivetrain, manipulator, distanceToSpeaker),
        AutoUtils.followPathAndIntake(W3_W2, manipulator),
        AutoUtils.distanceShoot(drivetrain, manipulator, distanceToSpeaker),
        AutoUtils.followPathAndIntake(W2_W1, manipulator),
        AutoUtils.distanceShoot(drivetrain, manipulator, distanceToSpeaker));
  }
}
