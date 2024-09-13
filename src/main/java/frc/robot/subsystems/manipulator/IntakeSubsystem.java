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

public class IntakeSubsystem extends SubsystemBase {

  /*
   * whether the shuffleboard is enabled or not
   */
  private final boolean SHUFFLEBOARD_ENABLED = false;

  /*
   * the object for the intake motor
   */
  private CANSparkMax intakeMotor;

  /*
   *the relative encoder on the intake motor
   */
  private RelativeEncoder intakeMotorEncoder;

  /*
   *
   */
  private final DigitalInput intakeBeamBreak =
      new DigitalInput(IntakeConstants.INTAKE_BEAM_BREAK_CHANNEL);

  /*
   * The constructor for the intake subsytem
   */
  public IntakeSubsystem() {
    configureMotors();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(RobotConstants.INTAKE_SYSTEM_TAB);
    }
  }

  // TODO: maybe remove and integrate into the constructor for the intake subsystem
  /*
   * sets up the motor for the intake
   */
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

  /*
   * Adds the speed of the intake and the output of the sensor into shuffleboard
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Intake Velocity", intakeMotorEncoder::getVelocity);
    shuffleboardTab.addBoolean("Intake Sensor Beam Break", intakeBeamBreak::get);
  }

  /*
   * checks to see if there is a note in the intake
   */
  public boolean isNoteInRoller() {
    return !intakeBeamBreak.get();
  }

  /*
   * sets the speed of the intake motor
   */
  public void setIntake(IntakeSpeed speed) {
    intakeMotor.set(speed.getSpeed());
  }

  /*
   * stop intake
   */

  public void stop() {
    intakeMotor.set(0);
  }

  /*
   * drop note by reversing intake
   */

  public void drop() {
    setIntake(IntakeSpeed.REVERSE);
  }

  /*
   * The enum for all possible speed values of the intake motor
   */
  public enum IntakeSpeed {
    REVERSE(-1),
    OFF(0),
    TENTH(0.1),
    QUARTER(0.25),
    HALF(0.5),
    FULL(1);

    private final double speed;

    /**
     * Constructor for IntakeSpeed.
     *
     * @param speed The speed value associated with the intake speed.
     */
    private IntakeSpeed(double speed) {
      this.speed = speed;
    }

    /**
     * Get the speed value associated with the intake speed.
     *
     * @return The speed value.
     */
    public double getSpeed() {
      return speed;
    }
  }
}
