package frc.robot.constants;

import com.pathplanner.lib.util.PIDConstants;

public class AutoConstants {
  public class HolonomicPathFollower {
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(5);
    public static final PIDConstants ROTATIONAL_PID_CONSTANTS = new PIDConstants(5);
  }
}
