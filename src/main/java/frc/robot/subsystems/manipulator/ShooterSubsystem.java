package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.ShooterConstants;
import frc.robot.constants.manipulator.ShooterConstants.ShooterVelocity;

public class ShooterSubsystem extends SubsystemBase {
  private final boolean SHUFFLEBOARD_ENABLED = true;

  private CANSparkFlex upperShooterMotor;
  private CANSparkFlex lowerShooterMotor;

  private RelativeEncoder upperShooterMotorEncoder;
  private RelativeEncoder lowerShooterMotorEncoder;

  private double targetVelocity;

  public ShooterSubsystem() {
    configureMotors();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(RobotConstants.SHOOTER_SYSTEM_TAB);
    }
  }

  /** Configure the shooter motors */
  private void configureMotors() {
    MotorPidBuilder UpperShooterMotorPidBuilder =
        new MotorPidBuilder()
            .withP(ShooterConstants.UpperShooterMotor.MotorPid.P)
            .withFf(ShooterConstants.UpperShooterMotor.MotorPid.Ff);

    MotorPidBuilder lowerShooterMotorPidBuilder =
        new MotorPidBuilder()
            .withP(ShooterConstants.LowerShooterMotor.MotorPid.P)
            .withFf(ShooterConstants.LowerShooterMotor.MotorPid.Ff);

    MotorBuilder upperShooterMotorConfig =
        new MotorBuilder()
            .withName(ShooterConstants.UpperShooterMotor.NAME)
            .withMotorPort(ShooterConstants.UpperShooterMotor.MOTOR_PORT)
            .withMotorInverted(ShooterConstants.UpperShooterMotor.INVERTED)
            .withCurrentLimit(ShooterConstants.UpperShooterMotor.CURRENT_LIMT)
            .withMotorPid(UpperShooterMotorPidBuilder)
            .withIdleMode(IdleMode.kCoast);

    MotorBuilder lowerShooterMotorConfig =
        new MotorBuilder()
            .withName(ShooterConstants.LowerShooterMotor.NAME)
            .withMotorPort(ShooterConstants.LowerShooterMotor.MOTOR_PORT)
            .withMotorInverted(ShooterConstants.LowerShooterMotor.INVERTED)
            .withCurrentLimit(ShooterConstants.LowerShooterMotor.CURRENT_LIMT)
            .withMotorPid(lowerShooterMotorPidBuilder)
            .withIdleMode(IdleMode.kCoast);

    upperShooterMotor =
        new CANSparkFlex(
            upperShooterMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    lowerShooterMotor =
        new CANSparkFlex(
            lowerShooterMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    upperShooterMotorEncoder = upperShooterMotor.getEncoder();
    lowerShooterMotorEncoder = lowerShooterMotor.getEncoder();

    MotorConfig.fromMotorConstants(
            upperShooterMotor, upperShooterMotorEncoder, upperShooterMotorConfig)
        .configureMotor()
        .configureEncoder()
        .burnFlash();

    MotorConfig.fromMotorConstants(
            lowerShooterMotor, lowerShooterMotorEncoder, lowerShooterMotorConfig)
        .configureMotor()
        .configureEncoder()
        .burnFlash();
  }

  /**
   * Setup the shuffleboard tab
   *
   * @param shuffleboardTab The shuffleboard tab
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Upper Shooter Vel", upperShooterMotorEncoder::getVelocity);
    shuffleboardTab.addDouble("Lower Shooter Vel", lowerShooterMotorEncoder::getVelocity);

    shuffleboardTab.addDouble("Upper Shooter Cur", upperShooterMotor::getOutputCurrent);
    shuffleboardTab.addDouble("Lower Shooter Cur", lowerShooterMotor::getOutputCurrent);

    shuffleboardTab.addDouble("Upper Shooter Temp", upperShooterMotor::getMotorTemperature);
    shuffleboardTab.addDouble("Lower Shooter Temp", lowerShooterMotor::getMotorTemperature);

    shuffleboardTab.addDouble("Target Velocity", () -> targetVelocity);
    shuffleboardTab.addBoolean("At Target Velocity?", this::atTargetVelocity);
  }

  /**
   * Check if the shooter motor is at the target velocity within a specified tolerance.
   *
   * @return True if the shooter motor is at the target velocity, false otherwise.
   */
  public boolean atTargetVelocity() {
    return targetVelocity
            - (lowerShooterMotorEncoder.getVelocity() + upperShooterMotorEncoder.getVelocity()) / 2
        < ShooterConstants.VELOCITY_TOLERANCE;
  }

  /**
   * Set the target velocity for the shooting motor and adjust the PID controller accordingly.
   *
   * @param velocity The desired velocity for the shooter motor.
   */
  public void set(ShooterVelocity velocity) {
    upperShooterMotor
        .getPIDController()
        .setReference(velocity.getVelocity(), CANSparkMax.ControlType.kVelocity);
    lowerShooterMotor
        .getPIDController()
        .setReference(velocity.getVelocity(), CANSparkMax.ControlType.kVelocity);
  }

  /** Stop both shooter motors. */
  public void stopMotor() {
    upperShooterMotor.stopMotor();
    lowerShooterMotor.stopMotor();
  }
}
