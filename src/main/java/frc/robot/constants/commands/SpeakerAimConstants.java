package frc.robot.constants.commands;

import edu.wpi.first.math.geometry.Rotation2d;

public class SpeakerAimConstants {
  public static final Rotation2d SETPOINT_TOLERANCE = Rotation2d.fromDegrees(1);
  public static final Rotation2d ANGULAR_OFFSET = Rotation2d.fromDegrees(180);

  public static class RotationPid {
    public static final double P = 5;

    public static class ContinuousInput {
      public static final double MIN = -Math.PI;
      public static final double MAX = Math.PI;
    }
  }
}
