package frc.robot.constants.manipulator;

public abstract class ElevatorConstants {
  public static final int UPPER_LIMIT_SWITCH_CHANNEL = 3;
  public static final int LOWER_LIMIT_SWITCH_CHANNEL = 2;

  public static final double HOMING_SPEED = -0.5;
  public static final double POSITION_TOLERANCE = 0.25;

  public static class ElevatorMotor {
    public static final String NAME = "Elevator Motor";
    public static final int MOTOR_PORT = 4;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;

    public static final int FORWARD_SOFT_LIMIT = 43;
    public static final int REVERSE_SOFT_LIMIT = 0;

    public static class MotorPid {
      public static final double P = 0.1;
    }
  }

  public enum ElevatorPosition {
    HOME(0);

    private double position;

    private ElevatorPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}
