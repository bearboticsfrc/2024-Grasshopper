package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.fms.AllianceColor;
import frc.bearbotics.fms.AllianceReadyListener;
import frc.robot.constants.CANdleConstants;

public class CANdleSubsystem extends SubsystemBase implements AllianceReadyListener {
  private CANdle CANdle;
  private Color currentColor;

  /**
   * Initializes a new instance of the Candle Subsystem, configuring the CANdle device with default
   * settings and preparing the LEDs for control.
   */
  public CANdleSubsystem() {
    configureCANdle();
    AllianceColor.addListener(this);
  }

  private void configureCANdle() {
    CANdle = new CANdle(CANdleConstants.CANDLE_PORT);

    ErrorCode errorCode = CANdle.configAllSettings(new CANdleConfiguration(), 100);
    if (!errorCode.equals(ErrorCode.OK)) {
      DriverStation.reportError("CANdle Error -> " + errorCode, false);
    }
  }

  /** updates the current alliance */
  @Override
  public void updateAlliance(Alliance alliance) {
    Color allianceColor = getColorFromAlliance(alliance);

    if (currentColor != allianceColor) {
      setColor(allianceColor);
    }
  }

  /**
   * Get the respective color from the alliance
   *
   * @param alliance The alliance
   * @return A {@link Color} describing the alliance color
   */
  private Color getColorFromAlliance(Alliance alliance) {
    return alliance == Alliance.Red ? Color.kRed : Color.kBlue;
  }

  /** Clears the animations and turns the LEDs off. */
  public void clear() {
    CANdle.clearAnimation(0);
    setColor(Color.kBlack);
  }

  /**
   * Sets the color of the entire LED strip to the specified color.
   *
   * @param color the color of the leds
   */
  public void setColor(Color color) {
    currentColor = color;
    CANdle.setLEDs(
        (int) (currentColor.red * 255),
        (int) (currentColor.green * 255),
        (int) (currentColor.blue * 255));
  }

  /**
   * Sets a specific animation pattern with a specified color. Supports strobe and larson patterns.
   *
   * @param pattern The animation to display on the LEDs.
   * @param color The color to use for the animation.
   */
  public void setPattern(CandlePattern pattern, Color color) {
    clear();

    switch (pattern) {
      case STROBE:
        setStrobeAnimation(color, CANdleConstants.STROBE_DURATION);
        break;
      case LARSON:
        setLarsonAnimation(color, CANdleConstants.LARSON_DURATION);
    }
  }

  /**
   * Sets the LEDs to a Larson animation.
   *
   * @param color The color of the LEDs.
   * @param speed The speed of the animation.
   */
  private void setLarsonAnimation(Color color, double speed) {
    currentColor = color;

    LarsonAnimation animation =
        new LarsonAnimation(
            (int) (color.red * 255),
            (int) (color.green * 255),
            (int) (color.blue * 255),
            0,
            speed,
            400,
            BounceMode.Back,
            1);

    setAnimation(animation, 0);
  }

  /**
   * Applies a strobe animation to the LEDs.
   *
   * @param color The color of the strobe effect.
   * @param speed The speed of the strobe effect.
   */
  private void setStrobeAnimation(Color color, double speed) {
    currentColor = color;

    StrobeAnimation animation =
        new StrobeAnimation(
            (int) (color.red * 255),
            (int) (color.green * 255),
            (int) (color.blue * 255),
            0,
            speed,
            400);

    setAnimation(animation, 0);
  }

  /** Animates the LEDs. */
  private void setAnimation(Animation animation, int slot) {
    CANdle.animate(animation, 0);
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
