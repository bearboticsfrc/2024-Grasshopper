package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.function.DoubleSupplier;

public class Sub2W2C3C4 implements AutoInterface {
  private final PathPlannerPath SUB2_W2 = PathPlannerPath.fromPathFile("Sub2W2");
  private final PathPlannerPath W2_C3 = PathPlannerPath.fromPathFile("W2C3");
  private final PathPlannerPath C3_SHOOT = PathPlannerPath.fromPathFile("C3Shoot");
  private final PathPlannerPath SHOOT_C4 = PathPlannerPath.fromPathFile("ShootC4");
  private final PathPlannerPath C4_SHOOT = PathPlannerPath.fromPathFile("C4Shoot");

  /**
   * Retrieve the name of this auto
   *
   * @return The auto name
   */
  public String getName() {
    return "Sub2W2C3C4";
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
        AutoUtils.followPathAndIntake(SUB2_W2, manipulator),
        AutoUtils.distanceShoot(drivetrain, manipulator, distanceToSpeaker),
        AutoUtils.followPathAndIntake(W2_C3, manipulator),
        AutoBuilder.followPath(C3_SHOOT),
        AutoUtils.distanceShoot(drivetrain, manipulator, distanceToSpeaker),
        AutoUtils.followPathAndIntake(SHOOT_C4, manipulator),
        AutoBuilder.followPath(C4_SHOOT),
        AutoUtils.distanceShoot(drivetrain, manipulator, distanceToSpeaker));
  }
}
