package frc.robot.constants;

import com.pathplanner.lib.util.PIDConstants;
import frc.robot.commands.autos.AutoInterface;
import frc.robot.commands.autos.PreloadShoot;
import frc.robot.commands.autos.Sub1W1;
import frc.robot.commands.autos.Sub2W2;
import frc.robot.commands.autos.Sub2W2C3C4;
import frc.robot.commands.autos.Sub2W3W2W1;
import frc.robot.commands.autos.Sub3W3;
import java.util.List;

public class AutoConstants {
  public static final List<AutoInterface> autos =
      List.of(
          new Sub1W1(),
          new Sub2W2(),
          new Sub3W3(),
          new Sub2W2C3C4(),
          new Sub2W3W2W1(),
          new PreloadShoot());

  public class HolonomicPathFollower {
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(1);
    public static final PIDConstants ROTATIONAL_PID_CONSTANTS = new PIDConstants(5);
  }
}
