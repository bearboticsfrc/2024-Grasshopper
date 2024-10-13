package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.function.DoubleSupplier;

public class PreloadShoot implements AutoInterface {
  /**
   * Retrieve the name of this auto.
   *
   * @return The auto name.
   */
  public String getName() {
    return "Preload Shoot";
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
    return manipulator.subwooferShoot();
  }
}
