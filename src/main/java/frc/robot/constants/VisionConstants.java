package frc.robot.constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {

  public static final Transform3d CAMERA_TO_ROBOT =
      new Transform3d(
          new Translation3d(114.299 / 1000, 0, 277.757 / 1000),
          new Rotation3d(0, 0, Math.toRadians(0)));

  public static final String CAMERA_NAME = "Front Camera";

  public static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEVS =
      MatBuilder.fill(Nat.N3(), Nat.N1(), 1, 1, 1 * Math.PI);

  public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.09;
  public static final double APRILTAG_CULL_DISTANCE = 4.0;

  public enum TAG {
    BLUE_SOURCE_RIGHT(1),
    BLUE_SOURCE_LEFT(2),
    RED_SPEAKER_RIGHT(3),
    RED_SPEAKER_CENTER(4),
    RED_AMP(5),
    BLUE_AMP(6),
    BLUE_SPEAKER_CENTER(7),
    BLUE_SPEAKER_LEFT(8),
    RED_SOURCE_RIGHT(9),
    RED_SOURCE_LEFT(10),
    RED_STAGE_LEFT(11),
    RED_STAGE_RIGHT(12),
    RED_STAGE_CENTER(13),
    BLUE_STAGE_CENTER(14),
    BLUE_STAGE_LEFT(15),
    BLUE_STAGE_RIGHT(16);

    public final int tagNumber;

    public int getValue() {
      return tagNumber;
    }

    private TAG(int tagNumber) {
      this.tagNumber = tagNumber;
    }
  }
}
