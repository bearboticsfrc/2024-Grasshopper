package frc.bearbotics.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DriveConstants;
import java.util.function.Supplier;

/**
 * The {@code ProcessedJoystick} class provides an interface for accessing input from an Xbox
 * controller, with utility methods to retrieve processed joystick axis data. The class is designed
 * to work with a {@link CommandXboxController} and applies a {@link ThrottleProfile} to adjust the
 * input values according to the current throttle setting.
 */
public class ProcessedJoystick {

  /** The Xbox controller providing the joystick input. */
  private final CommandXboxController joystick;

  /** Supplier function providing the current throttle profile. */
  private Supplier<ThrottleProfile> throttleProfileSupplier;

  /**
   * Constructs a new {@code ProcessedJoystick} object.
   *
   * @param joystick The {@link CommandXboxController} from which joystick inputs are read.
   * @param throttleProfile A {@link Supplier} that provides the current {@link ThrottleProfile},
   *     determining how the input is scaled.
   */
  public ProcessedJoystick(
      CommandXboxController joystick, Supplier<ThrottleProfile> throttleProfile) {
    this.joystick = joystick;
    this.throttleProfileSupplier = throttleProfile;
  }

  /**
   * Retrieves and processes the joystick input from the specified axis.
   *
   * <p>The input is scaled based on the throttle profile's maximum speed. The raw joystick input is
   * multiplied by the throttle profile's max speed and negated to align with expected directional
   * behavior.
   *
   * @param axis The {@link JoystickAxis} to read from the controller.
   * @return The processed input value, adjusted based on the current {@link ThrottleProfile}.
   */
  public double get(JoystickAxis axis) {
    double rawInput;

    switch (axis) {
      case Ly:
        rawInput = joystick.getLeftY();
        break;
      case Lx:
        rawInput = joystick.getLeftX();
        break;
      case Ry:
        rawInput = -joystick.getRightY();
        break;
      case Rx:
        rawInput = -joystick.getRightX();
        break;
      default:
        rawInput = 0;
    }

    // Adjust the raw input based on the throttle profile's max speed
    return -(rawInput * throttleProfileSupplier.get().getMaxSpeed());
  }

  /**
   * Enum representing the various joystick axes on the Xbox controller.
   *
   * <p>This enum allows for clearer, more readable code when specifying which axis input to
   * retrieve.
   */
  public enum JoystickAxis {
    /** Left Y-axis (up-down movement of the left joystick). */
    Ly,

    /** Left X-axis (left-right movement of the left joystick). */
    Lx,

    /** Right Y-axis (up-down movement of the right joystick). */
    Ry,

    /** Right X-axis (left-right movement of the right joystick). */
    Rx;
  }

  // Enum for different throttle modes
  public enum ThrottleProfile {
    TURBO(2),
    NORMAL(1),
    TURTLE(0.1);

    private final double MAX_VELOCITY = DriveConstants.MAX_VELOCITY;

    private final double maxSpeedMultiplier;

    private ThrottleProfile(double maxSpeedMultiplier) {
      this.maxSpeedMultiplier = maxSpeedMultiplier;
    }

    /**
     * Gets the maximum speed multiplier for the speed mode.
     *
     * @return The maximum speed multiplier.
     */
    public double getMaxSpeedMultiplier() {
      return maxSpeedMultiplier;
    }

    /**
     * Gets the maximum speed for the speed mode.
     *
     * @return The maximum speed.
     */
    public double getMaxSpeed() {
      return MAX_VELOCITY * getMaxSpeedMultiplier();
    }
  }
}
