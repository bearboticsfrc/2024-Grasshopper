package frc.robot.tests;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CANdleSubsystem.CANdlePattern;

public class SubsystemTester extends Command {
  private final TestableInterface test;
  private final CANdleSubsystem candle;

  private boolean isFinished = false;

  public SubsystemTester(TestableInterface test, CANdleSubsystem candle) {
    this.test = test;
    this.candle = candle;
  }

  @Override
  public void initialize() {
    candle.setPattern(CANdlePattern.LARSON, Color.kYellow);

    try {
      test.run();
      strobeCandle(Color.kGreen, 1.5);
    } catch (AssertionError error) {
      DriverStation.reportError(error.getMessage(), error.getStackTrace());
      strobeCandle(Color.kRed, 1.5);
    } finally {
      isFinished = true;
    }
  }

  private void strobeCandle(Color color, double duration) {
    candle.setPattern(CANdlePattern.STROBE, color);
    Timer.delay(duration);
    candle.setAllianceColor();
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
