package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

/*
 * this is a necessary data type so we dont depart quantities of a vector
 * you could put it in a vector data type but I want names so this is cleaner
 */
public class CameraPoseResultantIdentity {
  private double yaw;
  private double timestamp;
  Transform2d transform;

  /*
   * constructs a new camera transform resultant identity
   */
  public CameraPoseResultantIdentity(Transform2d transform, double timestamp) {
    this.yaw = yaw;
    this.transform = transform;
    this.timestamp = timestamp;
  }

  public double getTimestampSeconds() {
    return timestamp;
  }

  public Pose2d getPose2d(Transform2d robotToCameraTransform2d) {
    return new Pose2d(
        (transform.plus(robotToCameraTransform2d)).getTranslation(),
        transform.plus(robotToCameraTransform2d).getRotation());
  }
}
