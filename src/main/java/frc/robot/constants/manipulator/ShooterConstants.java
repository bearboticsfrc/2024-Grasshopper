package frc.robot.constants.manipulator;

public abstract class ShooterConstants {
  public static final double VELOCITY_TOLERANCE = 100;

  public static class UpperShooterMotor {
    public static final String NAME = "Upper Shooter Motor";
    public static final int MOTOR_PORT = 2;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;

    public static class MotorPid {
      public static final double P = -1;
      public static final double Ff = -1;
    }
  }

  public static class LowerShooterMotor {
    public static final String NAME = "Lower Shooter Motor";
    public static final int MOTOR_PORT = 3;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;

    public static class MotorPid {
      public static final double P = -1;
      public static final double Ff = -1;
    }
  }

  public enum ShooterVelocity {
    OFF(0);

    private double velocity;

    private ShooterVelocity(double velocity) {
      this.velocity = velocity;
    }

    public double getVelocity() {
      return velocity;
    }
  }
}
