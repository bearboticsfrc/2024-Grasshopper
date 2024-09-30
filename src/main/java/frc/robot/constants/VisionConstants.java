package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
  public static final String CAMERA_NAME = "Front Camera";

  public static final Transform3d ROBOT_TO_CAMERA_TRANSFORM =
      new Transform3d(
          new Translation3d(0.114299, 0, 0.277757), new Rotation3d(0, Math.toRadians(-27), 0));

  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

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
