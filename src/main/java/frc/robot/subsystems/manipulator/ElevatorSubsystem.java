package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.bearbotics.motor.MotorSoftLimit;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.ElevatorConstants;
import frc.robot.constants.manipulator.ElevatorConstants.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
  private final boolean SHUFFLEBOARD_ENABLED = true;

  private CANSparkMax elevatorMotor;
  private RelativeEncoder elevatorMotorEncoder;

  private final DigitalInput lowerLimitSwitch =
      new DigitalInput(ElevatorConstants.LOWER_LIMIT_SWITCH_CHANNEL);
  private final DigitalInput upperLimitSwitch =
      new DigitalInput(ElevatorConstants.UPPER_LIMIT_SWITCH_CHANNEL);

  private ElevatorPosition targetPosition = ElevatorPosition.HOME;

  public ElevatorSubsystem() {
    configureMotors();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(RobotConstants.ELEVATOR_SYSTEM_TAB);
    }
  }

  /** Configures the motors */
  private void configureMotors() {
    MotorPidBuilder elevatorMotorPidBuilder =
        new MotorPidBuilder().withP(ElevatorConstants.ElevatorMotor.MotorPid.P);

    MotorSoftLimit elevatorMotorForwardSoftLimit =
        new MotorSoftLimit()
            .withDirection(SoftLimitDirection.kForward)
            .withLimit(ElevatorConstants.ElevatorMotor.FORWARD_SOFT_LIMIT);

    MotorSoftLimit elevatorMotorReverseSoftLimit =
        new MotorSoftLimit()
            .withDirection(SoftLimitDirection.kReverse)
            .withLimit(ElevatorConstants.ElevatorMotor.REVERSE_SOFT_LIMIT);

    MotorBuilder elevatorMotorConfig =
        new MotorBuilder()
            .withName(ElevatorConstants.ElevatorMotor.NAME)
            .withMotorPort(ElevatorConstants.ElevatorMotor.MOTOR_PORT)
            .withMotorInverted(ElevatorConstants.ElevatorMotor.INVERTED)
            .withCurrentLimit(ElevatorConstants.ElevatorMotor.CURRENT_LIMT)
            .withMotorPid(elevatorMotorPidBuilder)
            .withForwardSoftLimit(elevatorMotorForwardSoftLimit)
            .withReverseSoftLimit(elevatorMotorReverseSoftLimit)
            .withIdleMode(IdleMode.kBrake);

    elevatorMotor =
        new CANSparkMax(elevatorMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    elevatorMotorEncoder = elevatorMotor.getEncoder();

    MotorConfig.fromMotorConstants(elevatorMotor, elevatorMotorEncoder, elevatorMotorConfig)
        .configureMotor()
        .configurePid()
        .configureEncoder()
        .burnFlash();
  }

  /** Publishes telemetry to shuffleboard */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Elevator Motor Encoder Position", elevatorMotorEncoder::getPosition);
    shuffleboardTab.addDouble("Elevator Motor Temperature", elevatorMotor::getMotorTemperature);
    shuffleboardTab.addDouble("Elevator Motor Output Current", elevatorMotor::getOutputCurrent);

    shuffleboardTab.addBoolean("Is At Target Position?", this::atTargetPosition);
    shuffleboardTab.addBoolean("Is At Lower Limit?", this::isAtLowerLimit);
    shuffleboardTab.addBoolean("Is At Upper Limit?", this::isAtUpperLimit);
  }

  @Override
  public void periodic() {
    // Over-extension protection in conjunction with the motor soft limits
    if (isAtLowerLimit() && elevatorMotor.getAppliedOutput() < 0) {
      stopMotor();
    } else if (isAtUpperLimit() && elevatorMotor.getAppliedOutput() > 0) {
      stopMotor();
    }
  }

  /**
   * Set the target postition for the elevator driver
   *
   * @param position An enum representing the elevator position.
   */
  public void set(ElevatorPosition position) {
    targetPosition = position;

    elevatorMotor
        .getPIDController()
        .setReference(position.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  /**
   * Sets the speed of the elevator motor
   *
   * @param speed The speed of the elevator motor
   */
  public void set(double speed) {
    elevatorMotor.set(speed);
  }

  /** Stop the elevator motor */
  public void stopMotor() {
    elevatorMotor.stopMotor();
  }

  /**
   * Whether the lowwe limit switch is active, indicating the elevator motor is at it's lowest
   * position
   *
   * @return True if the limit switch is active, otherwise false
   */
  public boolean isAtLowerLimit() {
    return !lowerLimitSwitch.get();
  }

  /**
   * Whether the upper limit switch is active, indicating the elevator motor is at it's highest
   * position
   *
   * @return True if the limit switch is active, otherwise false
   */
  public boolean isAtUpperLimit() {
    return !upperLimitSwitch.get();
  }

  /**
   * Check if the elevator motor is at the target position within a specific tolerance
   *
   * @return True if the arm elevator motor is at its target position, false otherwise
   */
  public boolean atTargetPosition() {
    return Math.abs(targetPosition.getPosition() - elevatorMotorEncoder.getPosition())
        < ElevatorConstants.POSITION_TOLERANCE;
  }

  /** Max the encoder for bypassing the reverse soft limit. */
  public void maxEncoder() {
    elevatorMotorEncoder.setPosition(ElevatorConstants.ElevatorMotor.FORWARD_SOFT_LIMIT);
  }

  /** Set the encoder position to 0. */
  public void tareEncoder() {
    elevatorMotorEncoder.setPosition(0);
  }
}
