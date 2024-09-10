package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.manipulator.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final boolean SHUFFLEBOARD_ENABLED = true;

  private CANSparkMax angleMotor;
  private CANSparkFlex upperShootingMotor;
  private CANSparkFlex lowerShootingMotor;

  private RelativeEncoder upperShooterMotorEncoder;
  private RelativeEncoder lowerShooterMotorEncoder;

  private RelativeEncoder relativeAngleEncoder;

  private DigitalInput bottomLimitSwitchAngleMotor =
      new DigitalInput(ShooterConstants.BOTTOM_LIMIT_SWITCH_CHANNEL);
  // private DigitalInput topLimitSwitchAngleMotor = new
  // DigitalInput(ShooterConstants.UPPER_LIMIT_SWITCH_CHANNEL);

  private double targetVelocity;

  private double targetArmPosition;

  public ShooterSubsystem() {
    configureMotors();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(RobotConstants.SHOOTER_SYSTEM_TAB);
    }
  }

  private void configureMotors() {
    MotorPidBuilder upperShootingMotorPIDConfig =
        new MotorPidBuilder()
            .withP(ShooterConstants.UpperShootingMotor.MotorPid.P)
            .withFf(ShooterConstants.UpperShootingMotor.MotorPid.Ff);

    MotorPidBuilder lowerShootingMotorPIDConfig =
        new MotorPidBuilder()
            .withP(ShooterConstants.LowerShootingMotor.MotorPid.P)
            .withFf(ShooterConstants.LowerShootingMotor.MotorPid.Ff);

    MotorPidBuilder armMotorPidBuilder =
        new MotorPidBuilder()
            .withP(ShooterConstants.AngleMotor.MotorPid.P)
            .withD(ShooterConstants.AngleMotor.MotorPid.D)
            .withMinOutput(ShooterConstants.AngleMotor.MotorPid.MIN_OUTPUT)
            .withMaxOutput(ShooterConstants.AngleMotor.MotorPid.MAX_OUTPUT)
            .withPositionPidWrappingEnabled(
                ShooterConstants.AngleMotor.MotorPid.POSITION_WRAPPING_ENABLED)
            .withPositionPidWrappingMin(ShooterConstants.AngleMotor.MotorPid.POSITION_WRAPPING_MIN)
            .withPositionPidWrappingMax(ShooterConstants.AngleMotor.MotorPid.POSITION_WRAPPING_MAX);

    MotorBuilder upperShooterMotorConfig =
        new MotorBuilder()
            .withName(ShooterConstants.UpperShootingMotor.NAME)
            .withMotorPort(ShooterConstants.UpperShootingMotor.MOTOR_PORT)
            .withMotorInverted(ShooterConstants.UpperShootingMotor.INVERTED)
            .withCurrentLimit(ShooterConstants.UpperShootingMotor.CURRENT_LIMT)
            .withMotorPid(upperShootingMotorPIDConfig)
            .withIdleMode(IdleMode.kCoast);

    MotorBuilder lowerShooterMotorConfig =
        new MotorBuilder()
            .withName(ShooterConstants.LowerShootingMotor.NAME)
            .withMotorPort(ShooterConstants.LowerShootingMotor.MOTOR_PORT)
            .withMotorInverted(ShooterConstants.LowerShootingMotor.INVERTED)
            .withCurrentLimit(ShooterConstants.LowerShootingMotor.CURRENT_LIMT)
            .withMotorPid(lowerShootingMotorPIDConfig)
            .withIdleMode(IdleMode.kCoast);

    MotorBuilder angleMotorConfig =
        new MotorBuilder()
            .withName(ShooterConstants.AngleMotor.NAME)
            .withMotorPort(ShooterConstants.LowerShootingMotor.MOTOR_PORT)
            .withMotorInverted(ShooterConstants.LowerShootingMotor.INVERTED)
            .withCurrentLimit(ShooterConstants.LowerShootingMotor.CURRENT_LIMT)
            .withMotorPid(lowerShootingMotorPIDConfig)
            .withIdleMode(IdleMode.kBrake);

    upperShootingMotor =
        new CANSparkFlex(
            upperShooterMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    lowerShootingMotor =
        new CANSparkFlex(
            lowerShooterMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    angleMotor =
        new CANSparkMax(angleMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);

    upperShooterMotorEncoder = upperShootingMotor.getEncoder();
    lowerShooterMotorEncoder = lowerShootingMotor.getEncoder();

    relativeAngleEncoder = angleMotor.getEncoder();
    relativeAngleEncoder.getPosition();
  }

  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab.addDouble("Upper Shooter Vel", upperShooterMotorEncoder::getVelocity);
    shuffleboardTab.addDouble("Lower Shooter Vel", lowerShooterMotorEncoder::getVelocity);

    shuffleboardTab.addDouble("Upper Shooter Cur", upperShootingMotor::getOutputCurrent);
    shuffleboardTab.addDouble("Lower Shooter Cur", lowerShootingMotor::getOutputCurrent);

    shuffleboardTab.addDouble("Upper Shooter Temp", upperShootingMotor::getMotorTemperature);
    shuffleboardTab.addDouble("Lower Shooter Temp", lowerShootingMotor::getMotorTemperature);

    shuffleboardTab.addDouble("Target Velocity", () -> targetVelocity);
    shuffleboardTab.addBoolean("At Target Velocity?", this::atTargetVelocity);

    shuffleboardTab.addDouble("Angle motor position", relativeAngleEncoder::getPosition);
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
   * @return True if the arm angle driver is at its target position, false otherwise
   */
  public boolean atTargetPosition() {
    return (Math.abs(targetArmPosition - relativeAngleEncoder.getPosition())
        < ShooterConstants.AngleMotor.MotorPid.POSITION_TOLERANCE);
  }

  /**
   * Set the target velocity for the shooting motor and adjust the PID controller accordingly.
   *
   * @param velocity The desired velocity for the shooter motor.
   */
  public void setShooter(double velocity) {
    upperShootingMotor.getPIDController().setReference(velocity, CANSparkMax.ControlType.kVelocity);
    lowerShootingMotor.getPIDController().setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  /** set the target postition for the angle driver */

  /** Stop both shooting motors. */
  public void setPosition(double position) {
    angleMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void stopShooter() {
    upperShootingMotor.stopMotor();
    lowerShootingMotor.stopMotor();
  }

  public void stopAngleMotor() {
    angleMotor.stopMotor();
  }

  public void stop() {
    stopShooter();
    stopAngleMotor();
  }

  public enum ShooterVelocity {
    SUBWOOFER_SHOOT(0),
    PODIUM_SHOOT(0),
    BLOOP_SHOOT(0),
    AMP_SHOOT(0),
    STAGE_SHOOT(0);

    private double velocity;

    private ShooterVelocity(double velocity) {
      this.velocity = velocity;
    }

    public double getVelocity() {
      return velocity;
    }
  }

  public enum AnglePosition {
    SUBWOOFER_SHOOT(0),
    PODIUM_SHOOT(0),
    BLOOP_SHOOT(0),
    AMP_SHOOT(0),
    STAGE_SHOOT(0);

    private double position;

    private AnglePosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}
