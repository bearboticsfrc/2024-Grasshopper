package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.constants.manipulator.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final boolean SHUFFLEBOARD_ENABLED = true;

  private CANSparkMax angleMotor;
  private CANSparkMax shootingMotor;

  private RelativeEncoder relativeAngleEncoder;

  private double targetVelocity;

  private void configureMotors() {
    MotorPidBuilder shootingMotorPIDConfig =
        new MotorPidBuilder()
            .withP(ShooterConstants.ShootingMotor.MotorPid.P)
            .withFf(ShooterConstants.ShootingMotor.MotorPid.Ff);
  }
}
