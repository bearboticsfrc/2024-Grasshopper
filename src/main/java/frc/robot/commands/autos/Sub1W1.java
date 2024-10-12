package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.function.DoubleSupplier;

public class Sub1W1 implements AutoInterface {
  private static final PathPlannerPath SUB1_W1 = PathPlannerPath.fromPathFile("Sub1W1");

  /**
   * Retrieve the name of this auto.
   *
   * @return The auto name.
   */
  public String getName() {
    return "Sub1W1";
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
        manipulator.subwooferShoot(),
        AutoUtils.followPathAndIntake(SUB1_W1, manipulator),
        AutoUtils.distanceShoot(drivetrain, manipulator, distanceToSpeaker));
  }
}
