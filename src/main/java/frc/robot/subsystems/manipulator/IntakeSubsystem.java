package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorConfig;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.IntakeConstants;
import frc.robot.constants.manipulator.IntakeConstants.IntakeVelocity;

public class IntakeSubsystem extends SubsystemBase {
  private final boolean SHUFFLEBOARD_ENABLED = true;

  private CANSparkMax intakeMotor;
  private RelativeEncoder intakeMotorEncoder;

  private final DigitalInput rollerBeamBreak =
      new DigitalInput(IntakeConstants.ROLLER_BEAM_BREAK_CHANNEL);
  private final DigitalInput shooterBeamBreak =
      new DigitalInput(IntakeConstants.SHOOTER_BEAM_BREAK_CHANNEL);

  /** The constructor for the intake subsytem */
  public IntakeSubsystem() {
    configureMotors();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(RobotConstants.INTAKE_SYSTEM_TAB);
    }
  }

  /** Configures the motor for the intake */
  private void configureMotors() {
    MotorBuilder intakeMotorBuilder =
        new MotorBuilder()
            .withName(IntakeConstants.IntakeMotor.NAME)
            .withMotorPort(IntakeConstants.IntakeMotor.MOTOR_PORT)
            .withMotorInverted(IntakeConstants.IntakeMotor.INVERTED)
            .withCurrentLimit(IntakeConstants.IntakeMotor.CURRENT_LIMT);

    intakeMotor =
        new CANSparkMax(intakeMotorBuilder.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    intakeMotorEncoder = intakeMotor.getEncoder();

    MotorConfig.fromMotorConstants(intakeMotor, intakeMotorEncoder, intakeMotorBuilder)
        .configureMotor()
        .configureEncoder()
        .burnFlash();
  }

  /** Publishes telemetry to shuffleboard */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Intake Encoder Velocity", intakeMotorEncoder::getVelocity);
    shuffleboardTab.addDouble("Intake Motor Temperature", intakeMotor::getMotorTemperature);

    shuffleboardTab.addBoolean("Is Note in Roller?", this::isNoteInRoller);
    shuffleboardTab.addBoolean("Is Note in Shooter?", this::isNoteInShooter);
  }

  /**
   * Whether the lower intake beam break is active, indicating the presence of a note.
   *
   * @return True if the beam break is active, otherwise false.
   */
  public boolean isNoteInRoller() {
    return !rollerBeamBreak.get();
  }

  /**
   * Whether the upper intake beam break is active, indicating the presence of a note.
   *
   * @return True if the beam break is active, otherwise false.
   */
  public boolean isNoteInShooter() {
    return shooterBeamBreak.get();
  }

  /**
   * Set the velocity of the intake motor
   *
   * @param position An enum representing the intake speed.
   */
  public void setVelocity(IntakeVelocity velocity) {
    intakeMotor.set(velocity.getSpeed());
  }

  /** Stop the intake motor */
  public void stopMotor() {
    intakeMotor.stopMotor();
  }
}
