package frc.bearbotics.util;

import java.time.LocalTime;

public class ParametricPoint {
  private LocalTime t;
  private double x;
  private double y;

  public ParametricPoint(LocalTime t, double x, double y) {
    this.t = t;
    this.x = x;
    this.y = y;
  }

  public LocalTime getT() {
    return t;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }
}
