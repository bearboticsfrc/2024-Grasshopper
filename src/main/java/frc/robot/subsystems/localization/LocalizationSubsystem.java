package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;

public class LocalizationSubsystem extends SubsystemBase {

  private LocalizingCamera camera =
      new LocalizingCamera(
          "camera", new PhotonCamera(VisionConstants.CAMERA_NAME), VisionConstants.CAMERA_TO_ROBOT);

  public LocalizationSubsystem() {

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    EstimationRunnable estimatorRunnable = new EstimationRunnable(camera.getNiceName(), camera);
  }
}
