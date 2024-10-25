package frc.bearbotics.tests;

import edu.wpi.first.wpilibj.Timer;

public class Testable {
  public void test(boolean assertion, String message) throws AssertionError {
    if (!assertion) {
      throw new AssertionError(message);
    }
  }

  public void wait(Runnable action, double waitSeconds) {
    action.run();
    Timer.delay(waitSeconds);
  }
}
