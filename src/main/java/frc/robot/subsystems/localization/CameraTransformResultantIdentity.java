package frc.robot.subsystems.localization;

/*
 * this is a necessary data type so we dont depart quantities of a vector
 * you could put it in a vector data type but I want names so this is cleaner
 */
public class CameraTransformResultantIdentity {

  private double dist;
  private double x;
  private double y;
  private double z;
  private double rot;

  /*
   * constructs a new camera transform resultant identity
   */
  public CameraTransformResultantIdentity(double dist, double x, double y, double z, double yaw) {
    this.dist = dist;
    this.x = x;
    this.y = y;
    this.z = z;
    this.rot = rot;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getZ() {
    return z;
  }

  public double getRot() {
    return rot;
  }
}
