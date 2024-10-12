package frc.robot.constants.manipulator;

import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

public abstract class ElevatorConstants {
  public static final int UPPER_LIMIT_SWITCH_CHANNEL = 3;
  public static final int LOWER_LIMIT_SWITCH_CHANNEL = 2;

  public static final double HOMING_SPEED = -0.5;
  public static final double POSITION_TOLERANCE = 0.75;

  public static final SortedMap<Double, Double> SHOOT_ANGLE_MAP =
      new TreeMap<>(
          Map.ofEntries(
              Map.entry(0.0, 20.0),
              Map.entry(1.106401, 22.0),
              Map.entry(1.641402, 17.5),
              Map.entry(2.191402, 12.0),
              Map.entry(2.721402, 9.0),
              Map.entry(3.171402, 6.5),
              Map.entry(3.611402, 5.0),
              Map.entry(4.221402, 3.55),
              Map.entry(4.571402, 1.8),
              Map.entry(4.971402, 1.3),
              Map.entry(5.0, 1.3)));

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
    HOME(0),
    SPEAKER(17),
    FEEDER(12.5);

    private double position;

    private ElevatorPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}
