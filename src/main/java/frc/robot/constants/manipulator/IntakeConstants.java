package frc.robot.constants.manipulator;

public abstract class IntakeConstants {
  public static final int ROLLER_BEAM_BREAK_CHANNEL = 0;
  public static final int SHOOTER_BEAM_BREAK_CHANNEL = 1;

  // Delay to wait until we stop the intake motors once we trip the beam-break in our intake cycle
  public static final double STOP_DELAY = 0;

  // Delay to wait until stopping the intake when feeding a note
  public static final double FEED_DELAY = 0.5;

  public static class IntakeMotor {
    public static final String NAME = "Intake Motor";
    public static final int MOTOR_PORT = 5;
    public static final int CURRENT_LIMT = 40;
    public static final boolean INVERTED = true;
  }

  /*
   * The enum for all possible speed values of the intake motor
   */
  public enum IntakeVelocity {
    REVERSE(-1),
    OFF(0),
    FULL(1),
    HALF(0.5);

    private final double speed;

    /**
     * Constructor for IntakeSpeed.
     *
     * @param speed The speed value associated with the intake speed.
     */
    private IntakeVelocity(double speed) {
      this.speed = speed;
    }

    /**
     * Get the speed value associated with the intake speed.
     *
     * @return The speed value.
     */
    public double getSpeed() {
      return speed;
    }
  }
}
