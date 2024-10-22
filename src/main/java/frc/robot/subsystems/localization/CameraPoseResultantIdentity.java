package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/*
 * this is a necessary data type so we dont depart quantities of a vector
 * you could put it in a vector data type but I want names so this is cleaner
 */
public class CameraPoseResultantIdentity {

  private double rot;
  private double timestamp;
  CoordinateTransform transform;

  /*
   * constructs a new camera transform resultant identity
   */
  public CameraPoseResultantIdentity(CoordinateTransform transform, double timestamp) {
    this.transform = transform;
    this.timestamp = timestamp;
  }

  public double getTimestampSeconds() {
    return timestamp;
  }

  public Pose2d getPose2d(Transform3d robotToCameraTransform) {
    return new Pose2d(
        (transform.getX() - robotToCameraTransform.getX()),
        transform.getY() - robotToCameraTransform.getY(),
        new Rotation2d(rot));
  }
}
