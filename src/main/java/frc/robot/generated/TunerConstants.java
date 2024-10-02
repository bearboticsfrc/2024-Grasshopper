package frc.robot.generated;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
  // Both sets of gains need to be tuned to your individual robot.

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final double kSlipCurrentA = 150.0;

  // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
  private static final TalonFXConfiguration driveInitialConfigs =
      new TalonFXConfiguration()
          .withClosedLoopRamps(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(1))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a relatively
                  // low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(100)
                  .withStatorCurrentLimitEnable(true));

  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a relatively
                  // low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(80)
                  .withStatorCurrentLimitEnable(true));

  private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  private static final Pigeon2Configuration pigeonConfigs = null;

  // Theoretical free speed (m/s) at 12v applied output;
  // This needs to be tuned to your individual robot
  public static final double kSpeedAt12VoltsMps = 4.95;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 3.5714285714285716;

  private static final double kDriveGearRatio = 6.122448979591837;
  private static final double kSteerGearRatio = 21.428571428571427;
  private static final double kWheelRadiusInches = 1.9;

  private static final boolean kInvertLeftSide = true;
  private static final boolean kInvertRightSide = false;

  private static final String kCANbusName = "drive";
  private static final int kPigeonId = 24;

  private static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANbusName(kCANbusName)
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(kSlipCurrentA)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withCANcoderInitialConfigs(cancoderInitialConfigs);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 5;
  private static final int kFrontLeftSteerMotorId = 6;
  private static final int kFrontLeftEncoderId = 11;
  private static final double kFrontLeftEncoderOffset = 0.08056640625;
  private static final boolean kFrontLeftSteerInvert = true;

  private static final double kFrontLeftXPosInches = 9.375;
  private static final double kFrontLeftYPosInches = 9.875;

  // Front Right
  private static final int kFrontRightDriveMotorId = 3;
  private static final int kFrontRightSteerMotorId = 4;
  private static final int kFrontRightEncoderId = 10;
  private static final double kFrontRightEncoderOffset = -0.351318359375;
  private static final boolean kFrontRightSteerInvert = true;

  private static final double kFrontRightXPosInches = 9.375;
  private static final double kFrontRightYPosInches = -9.875;

  // Back Left
  private static final int kBackLeftDriveMotorId = 7;
  private static final int kBackLeftSteerMotorId = 8;
  private static final int kBackLeftEncoderId = 12;
  private static final double kBackLeftEncoderOffset = 0.29296875;
  private static final boolean kBackLeftSteerInvert = true;

  private static final double kBackLeftXPosInches = -9.375;
  private static final double kBackLeftYPosInches = 9.875;

  // Back Right
  private static final int kBackRightDriveMotorId = 1;
  private static final int kBackRightSteerMotorId = 2;
  private static final int kBackRightEncoderId = 9;
  private static final double kBackRightEncoderOffset = -0.47802734375;
  private static final boolean kBackRightSteerInvert = true;

  private static final double kBackRightXPosInches = -9.375;
  private static final double kBackRightYPosInches = -9.875;

  private static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              Units.inchesToMeters(kFrontLeftXPosInches),
              Units.inchesToMeters(kFrontLeftYPosInches),
              kInvertLeftSide)
          .withSteerMotorInverted(kFrontLeftSteerInvert);
  private static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              Units.inchesToMeters(kFrontRightXPosInches),
              Units.inchesToMeters(kFrontRightYPosInches),
              kInvertRightSide)
          .withSteerMotorInverted(kFrontRightSteerInvert);
  private static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              Units.inchesToMeters(kBackLeftXPosInches),
              Units.inchesToMeters(kBackLeftYPosInches),
              kInvertLeftSide)
          .withSteerMotorInverted(kBackLeftSteerInvert);
  private static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              Units.inchesToMeters(kBackRightXPosInches),
              Units.inchesToMeters(kBackRightYPosInches),
              kInvertRightSide)
          .withSteerMotorInverted(kBackRightSteerInvert);

  public static final CommandSwerveDrivetrain DriveTrain =
      new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
}
