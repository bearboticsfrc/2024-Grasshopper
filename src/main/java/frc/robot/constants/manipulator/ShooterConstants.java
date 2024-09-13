package frc.robot.constants.manipulator;

public abstract class ShooterConstants {

  public static final double VELOCITY_TOLERANCE = 100;

  public static final int BOTTOM_LIMIT_SWITCH_CHANNEL = -1;

  /*
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
  */

  /*
  public enum AnglePosition {
    SUBWOOFER_SHOOT(0),
    PODIUM_SHOOT(0),
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
  */

  public static class UpperShootingMotor {
    public static final String NAME = "Upper Shooter Motor";
    // TODO: update the motor port value
    public static final int MOTOR_PORT = -1;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;

    public static class MotorPid {
      public static final double P = 0.000;
      public static final double Ff = 0.000;
    }
  }

  public static class LowerShootingMotor {
    public static final String NAME = "Lower Shooter Motor";
    // TODO: update the motor port value
    public static final int MOTOR_PORT = -1;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;

    public static class MotorPid {
      public static final double P = 0.000;
      public static final double Ff = 0.000;
    }
  }

  public static class AngleMotor {
    public static final String NAME = "angular shooter Motor";
    // TODO: update the motor port value
    public static final int MOTOR_PORT = -1;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;
    public static final double HOME_SPEED = -0.1;

    public static class MotorPid {
      public static final double P = 0.000;
      public static final double D = 0;

      public static final double MAX_OUTPUT = 0.5;
      public static final double MIN_OUTPUT = 0;

      public static final boolean POSITION_WRAPPING_ENABLED = true;

      public static final double POSITION_WRAPPING_MIN = 0;

      public static final double POSITION_WRAPPING_MAX = 0;

      public static final double POSITION_TOLERANCE = 5;
    }
  }
}
