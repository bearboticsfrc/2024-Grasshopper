package frc.constants.manipulator;

public abstract class ShooterConstants {

  public static class ShootingMotor {
    public static final String NAME = "Upper Shooter Motor";
    // TODO: update the motor port value
    public static final int MOTOR_PORT = -1;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = false;

    public static class MotorPid {
      public static final double P = 0.000;
      public static final double Ff = 0.000;
    }
  }
}
