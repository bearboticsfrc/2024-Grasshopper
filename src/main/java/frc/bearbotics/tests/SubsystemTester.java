package frc.bearbotics.tests;

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
  private boolean isSuccess = false;

  public SubsystemTester(TestableInterface test, CANdleSubsystem candle) {
    this.test = test;
    this.candle = candle;
  }

  @Override
  public void initialize() {
    candle.setPattern(CANdlePattern.LARSON, Color.kYellow);

    try {
      test.run();
      isSuccess = true;
    } catch (AssertionError error) {
      DriverStation.reportError(error.getMessage(), false);
      isSuccess = false;
    } finally {
      test.end();
      isFinished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (isSuccess) {
      strobeCandle(Color.kGreen, 1.5);
    } else {
      strobeCandle(Color.kRed, 1.5);
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
