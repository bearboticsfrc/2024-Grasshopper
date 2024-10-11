package frc.robot.constants.commands;

import edu.wpi.first.math.geometry.Rotation2d;

public class PoseAimConstants {
  public static final Rotation2d SETPOINT_TOLERANCE = Rotation2d.fromDegrees(2);
  public static final Rotation2d ANGULAR_OFFSET = Rotation2d.fromDegrees(180);

  public static class RotationPid {
    public static final double P = 8;
    public static final double I = 6;
    public static final double D = 1;

    public static class ContinuousInput {
      public static final double MIN = -Math.PI;
      public static final double MAX = Math.PI;
    }
  }
}
