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

  private CANSparkMax angleMotor;

  private RelativeEncoder angleMotorEncoder;

  private final DigitalInput bottomLimitSwitch =
      new DigitalInput(ElevatorConstants.LOWER_LIMIT_SWITCH_CHANNEL);
  private final DigitalInput upperLimitSwitch =
      new DigitalInput(ElevatorConstants.UPPER_LIMIT_SWITCH_CHANNEL);

  private double targetPosition;

  public ElevatorSubsystem() {
    configureMotors();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(RobotConstants.ANGLE_SYSTEM_TAB);
    }
  }

  private void configureMotors() {
    MotorPidBuilder angleMotorPidBuilder =
        new MotorPidBuilder()
            .withP(ElevatorConstants.ElevatorMotor.MotorPid.P)
            .withI(ElevatorConstants.ElevatorMotor.MotorPid.I)
            .withMinOutput(ElevatorConstants.ElevatorMotor.MotorPid.MIN_OUTPUT)
            .withMaxOutput(ElevatorConstants.ElevatorMotor.MotorPid.MAX_OUTPUT);

    MotorBuilder angleMotorConfig =
        new MotorBuilder()
            .withName(ElevatorConstants.ElevatorMotor.NAME)
            .withMotorPort(ElevatorConstants.ElevatorMotor.MOTOR_PORT)
            .withMotorInverted(ElevatorConstants.ElevatorMotor.INVERTED)
            .withCurrentLimit(ElevatorConstants.ElevatorMotor.CURRENT_LIMT)
            .withMotorPid(angleMotorPidBuilder)
            .withIdleMode(IdleMode.kBrake);

    angleMotor =
        new CANSparkMax(angleMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    angleMotorEncoder = angleMotor.getEncoder();

    MotorConfig.fromMotorConstants(angleMotor, angleMotorEncoder, angleMotorConfig)
        .configureMotor()
        .configureEncoder()
        .burnFlash();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Angle Motor Encoder Position", angleMotorEncoder::getPosition);
    shuffleboardTab.addDouble("Angle Motor Temperature", angleMotor::getMotorTemperature);
  }

  /**
   * Set the target postition for the angle driver
   *
   * @param position An enum representing the angle position.
   */
  public void set(ElevatorPosition position) {
    angleMotor
        .getPIDController()
        .setReference(position.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  /**
   * Sets the speed of the angle motor
   *
   * @param speed The speed of the angle motor
   */
  public void set(double speed) {
    angleMotor.set(speed);
  }

  /** Stop the angle motor */
  public void stopMotor() {
    angleMotor.stopMotor();
  }

  /**
   * Whether the bottom limit switch is active, indicating the angle motor is at it's lowest
   * position
   *
   * @return True if the limit switch is active, otherwise false
   */
  public boolean isAtLowerLimit() {
    return bottomLimitSwitch.get();
  }

  /**
   * Whether the upper limit switch is active, indicating the angle motor is at it's highest
   * position
   *
   * @return True if the limit switch is active, otherwise false
   */
  public boolean isAtUpperLimit() {
    return upperLimitSwitch.get();
  }

  /**
   * Check if the angle motor is at the target position within a specific tolerance
   *
   * @return True if the arm angle motor is at its target position, false otherwise
   */
  public boolean atTargetPosition() {
    return Math.abs(targetPosition - angleMotorEncoder.getPosition())
        < ElevatorConstants.POSITION_TOLERANCE;
  }
}
