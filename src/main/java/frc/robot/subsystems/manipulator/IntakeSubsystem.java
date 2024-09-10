package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.robot.constants.manipulator.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final boolean SHUFFLEBOARD_ENABLED = false;
  private CANSparkMax intakeMotor;
  private RelativeEncoder intakeMotorEncoder;
  private final DigitalInput intakeBeamBreak =
      new DigitalInput(IntakeConstants.INTAKE_BEAM_BREAK_CHANNEL);

  public IntakeSubsystem() {
    configureMotors();
  }

  private void configureMotors() {
    MotorBuilder intakeMotor =
        new MotorBuilder()
            .withName(IntakeConstants.IntakeMotor.NAME)
            .withMotorPort(IntakeConstants.IntakeMotor.MOTOR_PORT)
            .withMotorInverted(IntakeConstants.IntakeMotor.INVERTED)
            .withCurrentLimit(IntakeConstants.IntakeMotor.CURRENT_LIMT);
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Intake Velocity", intakeMotorEncoder::getVelocity);
    shuffleboardTab.addBoolean("Intake Sensor Beam Break", intakeBeamBreak::get);
  }

  public boolean isNoteInRoller() {
    return !intakeBeamBreak.get();
  }

  public void setRoller(IntakeSpeed speed) {
    intakeMotor.set(speed.getSpeed());
  }

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
