package frc.robot.subsystems.candle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.fms.AllianceColor;
import frc.bearbotics.fms.AllianceReadyListener;

public class CandleSubsystem extends SubsystemBase implements AllianceReadyListener {
  // TODO: set all of these values
  private final int CANDLE_PORT = -1;
  private final CANdle LEDS = new CANdle(CANDLE_PORT);
  private final int animationSlot = 0;
  private Color currentColor;
  private int ledsSize;

  /**
   * Initializes a new instance of the Candle Subsystem, configuring the CANdle device with default
   * settings and preparing the LED segment for control.
   */
  public CandleSubsystem() {
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    LEDS.configAllSettings(candleConfiguration, 100);

    AllianceColor.addListener(this);
  }

  @Override
  public void updateAlliance(Alliance alliance) {

    if (currentColor == null || currentColor == Color.kRed || currentColor == Color.kBlue) {
      setAllianceColor();
    }
  }

  /**
   * Clears the animations and sets the color of all LED segments to black (effectively turning them
   * off).
   */

  /**
   * Clears the animations and sets the color of a specific LED segment to black.
   *
   * @param segment The LED segment to clear.
   */
  public void clearSegment(CANdle segment) {
    LEDS.clearAnimation(animationSlot);
    setColor(Color.kBlack);
  }

  /** Sets the color of the entire LED strip. */
  public void setColor(Color color) {
    currentColor = color;
    LEDS.setLEDs(
        (int) (currentColor.red * 255),
        (int) (currentColor.green * 255),
        (int) (currentColor.blue * 255));
  }

  /**
   * Sets a specific animation pattern with a specified color for specified segment. Supports strobe
   * and larson patterns.
   *
   * @param pattern The pattern to display on the LEDs.
   * @param color The color to use for the pattern.
   * @param segment The segment to set the pattern to.
   */
  public void setPattern(CandlePattern pattern, Color color) {
    clearSegment(LEDS);

    switch (pattern) {
      case STROBE:
        setStrobeAnimation(color, 10);
        break;
      case LARSON:
        setLarsonAnimation(color, 0.001);
    }
  }

  public void setLarsonAnimation(Color color, double speed) {
    currentColor = color;

    LarsonAnimation animation =
        new LarsonAnimation(
            (int) (color.red * 255),
            (int) (color.green * 255),
            (int) (color.blue * 255),
            0,
            speed,
            ledsSize,
            BounceMode.Back,
            0);

    setAnimation(animation, animationSlot);
  }
  /**
   * Applies a strobe animation to this LED segment.
   *
   * @param color The color of the strobe effect.
   * @param speed The speed of the strobe effect.
   */
  public void setStrobeAnimation(Color color, double speed) {
    currentColor = color;

    StrobeAnimation animation =
        new StrobeAnimation(
            (int) (color.red * 255),
            (int) (color.green * 255),
            (int) (color.blue * 255),
            0,
            speed,
            ledsSize);

    setAnimation(animation, animationSlot);
  }

  /**
   * Sets the LED color based on the robot's alliance color. Intended to visually indicate the
   * alliance during competitions.
   */
  public void setAllianceColor() {
    Color color = getAllianceColor(AllianceColor.isRedAlliance());

    setColor(color);
  }

  private void setAnimation(Animation animation, int slot) {
    LEDS.animate(animation, animationSlot);
  }

  private Color getAllianceColor(boolean isRedAlliance) {
    return isRedAlliance ? Color.kRed : Color.kBlue;
  }

  public enum CandlePattern {
    /** A strobing effect, rapidly blinking the LEDs on and off. */
    STROBE,

    /**
     * A "Larson scanner" (or Knight Rider) effect, creating a moving dot back and forth across the
     * LEDs.
     */
    LARSON;
  }
}
