package frc.robot.subsystems.localization;

public class CoordinateTransform {
  private double y;
  private double x;
  private double r;
  private double theta;

  CoordinateTransform(double ipOne, double ipTwo, Boolean isCartesian) {
    if (isCartesian) {
      this.y = ipOne;
      this.x = ipTwo;
      this.r = Math.sqrt(Math.pow(ipOne, 2) + Math.pow(ipTwo, 2));
      this.theta = Math.atan(this.x / this.y);
      return;
    }
    this.r = ipOne;
    this.theta = ipTwo;
    this.x = this.r * Math.sin(this.theta);
    this.y = this.r * Math.cos(this.theta);
  }

  public double getY() {
    return this.y;
  }

  public double getX() {
    return this.x;
  }

  public double getR() {
    return this.r;
  }

  public double getTheta() {
    return this.theta;
  }
}
