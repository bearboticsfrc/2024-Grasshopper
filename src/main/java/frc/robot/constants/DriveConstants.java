package frc.robot.constants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  // Controller ports.
  public static final int DRIVER_CONTROLLER_PORT = 0;

  public static final double MAX_VELOCITY =
      TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed

  public static final double MAX_ANGULAR_VELOCITY =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public static final double LINEAR_DEADBAND = MAX_VELOCITY * 0.1; // 10% deadband
  public static final double ROTATIONAL_DEADBAND = MAX_ANGULAR_VELOCITY * 0.1; // 10% deadband

  // Field centric swerve request
  public static final SwerveRequest.FieldCentric FIELD_CENTRIC_SWERVE_REQUEST =
      new SwerveRequest.FieldCentric()
          .withDeadband(LINEAR_DEADBAND)
          .withRotationalDeadband(ROTATIONAL_DEADBAND)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public static final Telemetry LOGGER = new Telemetry();
}
