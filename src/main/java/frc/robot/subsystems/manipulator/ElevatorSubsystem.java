package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.bearbotics.motor.MotorPidBuilder;
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

  private double targetPosition;

  public ElevatorSubsystem() {
    configureMotors();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(RobotConstants.ELEVATOR_SYSTEM_TAB);
    }
  }

  /** Configures the motors */
  private void configureMotors() {
    MotorPidBuilder elevatorMotorPidBuilder =
        new MotorPidBuilder()
            .withP(ElevatorConstants.ElevatorMotor.MotorPid.P)
            .withI(ElevatorConstants.ElevatorMotor.MotorPid.I)
            .withMinOutput(ElevatorConstants.ElevatorMotor.MotorPid.MIN_OUTPUT)
            .withMaxOutput(ElevatorConstants.ElevatorMotor.MotorPid.MAX_OUTPUT);

    MotorBuilder elevatorMotorConfig =
        new MotorBuilder()
            .withName(ElevatorConstants.ElevatorMotor.NAME)
            .withMotorPort(ElevatorConstants.ElevatorMotor.MOTOR_PORT)
            .withMotorInverted(ElevatorConstants.ElevatorMotor.INVERTED)
            .withCurrentLimit(ElevatorConstants.ElevatorMotor.CURRENT_LIMT)
            .withMotorPid(elevatorMotorPidBuilder)
            .withIdleMode(IdleMode.kBrake);

    elevatorMotor =
        new CANSparkMax(elevatorMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    elevatorMotorEncoder = elevatorMotor.getEncoder();

    MotorConfig.fromMotorConstants(elevatorMotor, elevatorMotorEncoder, elevatorMotorConfig)
        .configureMotor()
        .configureEncoder()
        .burnFlash();
  }

  /** Publishes telemetry to shuffleboard */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Elevator Motor Encoder Position", elevatorMotorEncoder::getPosition);
    shuffleboardTab.addDouble("Elevator Motor Temperature", elevatorMotor::getMotorTemperature);

    shuffleboardTab.addBoolean("Is At Lower Limit?", this::isAtLowerLimit);
    shuffleboardTab.addBoolean("Is At Upper Limit?", this::isAtUpperLimit);
  }

  @Override
  public void periodic() {
    // Over-extension protection in conjunction with the motors soft limits
    if (isAtLowerLimit() && elevatorMotor.get() > 0) {
      stopMotor();
    } else if (isAtUpperLimit() && elevatorMotor.get() < 0) {
      stopMotor();
    }
  }

  /**
   * Set the target postition for the elevator driver
   *
   * @param position An enum representing the elevator position.
   */
  public void set(ElevatorPosition position) {
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
    return Math.abs(targetPosition - elevatorMotorEncoder.getPosition())
        < ElevatorConstants.POSITION_TOLERANCE;
  }
}
