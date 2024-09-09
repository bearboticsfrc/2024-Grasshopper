package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.constants.manipulator.IntakeConstants;

public class IntakeSubsystems extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private RelativeEncoder intakeMotorEncoder;

  public IntakeSubsystems() {
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
  }
}
