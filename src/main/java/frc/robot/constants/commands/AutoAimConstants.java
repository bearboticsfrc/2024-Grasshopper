package frc.robot.constants.commands;

import edu.wpi.first.math.geometry.Rotation2d;

public class AutoAimConstants {
  public static Rotation2d ANGULAR_OFFSET = Rotation2d.fromDegrees(180);

  public static class RotationPid {
    public static final double P = 4;
    public static final double I = 0.01;

    public static class ContinuousInput {
      public static final double MIN = -Math.PI;
      public static final double MAX = Math.PI;
    }
  }
}
