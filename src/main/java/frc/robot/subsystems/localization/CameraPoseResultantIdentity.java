package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/*
 * this is a necessary data type so we dont depart quantities of a vector
 * you could put it in a vector data type but I want names so this is cleaner
 */
public class CameraPoseResultantIdentity {

  private double dist;
  private double x;
  private double y;
  private double z;
  private double rot;
  private double timestamp;

  /*
   * constructs a new camera transform resultant identity
   */
  public CameraPoseResultantIdentity(
      double dist, double y, double x, double yaw, double timestamp) {
    this.dist = dist;
    this.x = x;
    this.y = y;
    this.rot = rot;
    this.timestamp = timestamp;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getRot() {
    return rot;
  }

  public double getTimestampSeconds() {
    return timestamp;
  }

  public Pose2d getPose2d(Transform3d robotToCameraTransform) {
    return new Pose2d(
        (x + robotToCameraTransform.getX()),
        y + robotToCameraTransform.getY(),
        new Rotation2d(rot));
  }
}
