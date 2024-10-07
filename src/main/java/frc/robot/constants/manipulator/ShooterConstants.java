package frc.robot.constants.manipulator;

import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

public abstract class ShooterConstants {
  public static final double VELOCITY_TOLERANCE = 75;

  public static final SortedMap<Double, Double> SHOOT_ANGLE_MAP =
      new TreeMap<>(
          Map.ofEntries(
              Map.entry(0.0, 2800.0),
              Map.entry(1.335, 2800.0),
              Map.entry(1.87, 2800.0),
              Map.entry(2.42, 3000.0),
              Map.entry(2.95, 3100.0),
              Map.entry(3.4, 3300.0),
              Map.entry(3.84, 3500.0),
              Map.entry(4.45, 4000.0),
              Map.entry(4.8, 4250.0)));

  public static class UpperShooterMotor {
    public static final String NAME = "Upper Shooter Motor";
    public static final int MOTOR_PORT = 2;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = true;

    public static class MotorPid {
      public static final double P = 0.0001;
      public static final double Ff = 0.000152;
    }
  }

  public static class LowerShooterMotor {
    public static final String NAME = "Lower Shooter Motor";
    public static final int MOTOR_PORT = 3;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;

    public static class MotorPid {
      public static final double P = 0.0001;
      public static final double Ff = 0.000150;
    }
  }

  public enum ShooterVelocity {
    OFF(0),
    SPEAKER(2800);

    private double velocity;

    private ShooterVelocity(double velocity) {
      this.velocity = velocity;
    }

    public double getVelocity() {
      return velocity;
    }
  }
}
