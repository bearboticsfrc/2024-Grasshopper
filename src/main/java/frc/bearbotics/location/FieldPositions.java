package frc.bearbotics.location;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.bearbotics.fms.AllianceColor;
import frc.robot.constants.VisionConstants;
import java.io.IOException;

public class FieldPositions {

  public static double FIELD_LENGTH = 16.541; // meters
  public static double FIELD_WIDTH = 8.211; // meters
  public static double TAG_OUTSIDE_FIELD = 0.04;

  private AprilTagFieldLayout layout;

  private static FieldPositions instance = null;

  public FieldPositions() {
    initializeLayout();
  }

  public static FieldPositions getInstance() {
    if (instance == null) {
      instance = new FieldPositions();
    }
    return instance;
  }

  private void initializeLayout() {
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
    }
  }

  public AprilTagFieldLayout getLayout() {
    return layout;
  }

  public double getWingY() {
    double wingY = 5.87;
    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      wingY = FIELD_LENGTH - 5.87;
    }
    return wingY;
  }

  public Pose2d getW1() {
    double y = (FIELD_WIDTH / 2.0) + (1.45 * 2.0);
    double x = 2.90;
    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      x = FIELD_LENGTH - x;
    }
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getW2() {
    double y = FIELD_WIDTH / 2.0 + 1.45;
    double x = 2.90;
    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      x = FIELD_LENGTH - x;
    }
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getW3() {
    double y = FIELD_WIDTH / 2.0;
    double x = 2.90;
    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      x = FIELD_LENGTH - x;
    }
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getC1() {
    double y = FIELD_WIDTH / 2.0 + (1.68 * 2.0);
    double x = FIELD_LENGTH / 2.0;
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getC2() {
    double y = FIELD_WIDTH / 2.0 + 1.68;
    double x = FIELD_LENGTH / 2.0;
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getC3() {
    double y = FIELD_WIDTH / 2.0;
    double x = FIELD_LENGTH / 2.0;
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getC4() {
    double y = FIELD_WIDTH / 2.0 - 1.68;
    double x = FIELD_LENGTH / 2.0;
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getC5() {
    double y = FIELD_WIDTH / 2.0 - (1.68 * 2.0);
    double x = FIELD_LENGTH / 2.0;
    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getSpeakerCenter() {
    int tagId = VisionConstants.TAG.BLUE_SPEAKER_CENTER.getValue();

    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      tagId = VisionConstants.TAG.RED_SPEAKER_CENTER.getValue();
    }
    return getTagPose(tagId).plus(getSpeakerOffset());
  }

  public Transform2d getSpeakerOffset() {
    Transform2d transform = new Transform2d(0.2, 0, new Rotation2d());

    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      transform = new Transform2d(-0.2, 0, new Rotation2d());
    }

    return transform;
  }

  public Pose2d getFeederPose() {
    Pose2d pose = new Pose2d(1.8796, 6.731, new Rotation2d());

    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      pose = new Pose2d(14.6304, 6.731, new Rotation2d());
    }

    return pose;
  }

  public Pose2d getAmp() {
    int tagId = VisionConstants.TAG.BLUE_AMP.getValue();

    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      tagId = VisionConstants.TAG.RED_AMP.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getSourceLeft() {
    int tagId = VisionConstants.TAG.BLUE_SOURCE_LEFT.getValue();

    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      tagId = VisionConstants.TAG.RED_SOURCE_LEFT.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getSourceRight() {
    int tagId = VisionConstants.TAG.BLUE_SOURCE_RIGHT.getValue();

    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      tagId = VisionConstants.TAG.RED_SOURCE_RIGHT.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getStageLeft() {
    int tagId = VisionConstants.TAG.BLUE_STAGE_LEFT.getValue();

    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      tagId = VisionConstants.TAG.RED_STAGE_LEFT.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getStageCenter() {
    int tagId = VisionConstants.TAG.BLUE_STAGE_CENTER.getValue();

    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      tagId = VisionConstants.TAG.RED_STAGE_CENTER.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getStageRight() {
    int tagId = VisionConstants.TAG.BLUE_STAGE_RIGHT.getValue();

    if (AllianceColor.getAlliance().equals(Alliance.Red)) {
      tagId = VisionConstants.TAG.BLUE_STAGE_RIGHT.getValue();
    }
    return getTagPose(tagId);
  }

  public Pose2d getTagPose(int tag) {
    return layout.getTagPose(tag).get().toPose2d();
  }
}
