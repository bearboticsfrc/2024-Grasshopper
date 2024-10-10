package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.function.DoubleSupplier;

public class Sub2W2 implements AutoInterface {
  private static final PathPlannerPath SUB2_W2 = PathPlannerPath.fromPathFile("Sub2W2");

  /**
   * Retrieve the name of this auto.
   *
   * @return The auto name.
   */
  public String getName() {
    return "Sub2W2";
  }

  /**
   * Retrieves a composite Command that represents the full autonomous routine.
   *
   * @param drivetrain The swerve drivetrain subsystem.
   * @param manipulator The manipulator subsystem.
   * @param distanceToSpeaker The supplier for distance to the speaker.
   * @return The autonomous command sequence.
   */
  public Command get(
      CommandSwerveDrivetrain drivetrain,
      ManipulatorSubsystem manipulator,
      DoubleSupplier distanceToSpeaker) {

    return Commands.sequence(
        manipulator.distanceShoot(distanceToSpeaker),
        AutoUtils.followPathAndIntake(SUB2_W2, manipulator),
        AutoUtils.distanceShoot(drivetrain, manipulator, distanceToSpeaker));
  }
}
