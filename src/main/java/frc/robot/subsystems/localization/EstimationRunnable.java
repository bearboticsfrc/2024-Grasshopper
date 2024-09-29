package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class EstimationRunnable implements Runnable {
  private final PhotonCamera photonCamera;
  private final AprilTagFieldLayout layout;
  private final AtomicReference<CameraPoseResultantIdentity> atomicEstimatedCameraTransform =
      new AtomicReference<>();

  public EstimationRunnable(String name, LocalizingCamera camera) {

    this.layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    this.layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonCamera = camera.getPhotonCamera();
  }

  /**
   * The main run method executed by the runnable. It retrieves the latest results from the
   * PhotonCamera, filters out targets based on ambiguity and distance, and updates the estimated
   * robot pose accordingly. Only valid poses within the field boundaries are considered.
   */
  @Override
  public void run() {
    PhotonPipelineResult photonResults = photonCamera.getLatestResult();

    /*
    *  photonResults.targets.removeIf(
         t ->
             t.getPoseAmbiguity() > VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD
                 || isTargetTooFarAway(t));\][]
    */

    if (!photonResults.hasTargets()) return;

    atomicEstimatedCameraTransform.set(getTransform(photonResults));
  }

  public double getHypotenuse(PhotonTrackedTarget target) {
    Pose3d tagPose = layout.getTagPose(target.getFiducialId()).get();
    double individualZ = tagPose.getZ() - VisionConstants.CAMERA_TO_ROBOT.getZ();
    return individualZ / Math.tan(Math.toRadians(target.getPitch() + 27));
  }

  public CameraPoseResultantIdentity getTransform(PhotonPipelineResult result) {
    int count = 0;
    double y = 0;
    double x = 0;

    double yaw = 0;

    List<PhotonTrackedTarget> targets = result.targets;
    double maxHype = 0;
    List<PhotonTrackedTarget> sortedTargets = new ArrayList<PhotonTrackedTarget>();
    for (PhotonTrackedTarget i : targets) {
      double indHype = getHypotenuse(i);
      if (indHype > maxHype) {
        sortedTargets.add(i);
      } else {
        for (PhotonTrackedTarget a : sortedTargets) {
          if (indHype <= getHypotenuse(a)) {
            sortedTargets.add(sortedTargets.indexOf(a), i);
          }
        }
      }
    }

    // Arrays.sort(targets, (e1, e2) -> Comparator.comparing(() -> getHypotenuse(e1), () ->
    // getHypotenuse(e2)));
    double lowestDistance = getHypotenuse(sortedTargets.get(0));
    for (PhotonTrackedTarget i : (sortedTargets)) {

      Pose3d tagPose = layout.getTagPose(i.getFiducialId()).get();
      double individualDist = getHypotenuse(i);
      if (((individualDist - lowestDistance) > 1)) {
        break;
      }

      count += 1;

      double nueralX = i.getBestCameraToTarget().getX();
      double nueralY = i.getBestCameraToTarget().getY();
      double nueralDist = Math.hypot(nueralX, nueralY);
      double scalar = nueralDist / individualDist;
      double indX = nueralX / scalar;
      double indY = nueralY / scalar;

      yaw += tagPose.getRotation().getZ() + i.getYaw();

      // DataLogManager.log("")
      y += tagPose.getY() + indY;
      x += tagPose.getX() + indX;
      DataLogManager.log(Double.toString(individualDist));
    }
    y /= count;
    x /= count;
    yaw /= count;
    double time = result.getTimestampSeconds();

    return new CameraPoseResultantIdentity(y, x, yaw, time);
  }

  public CameraPoseResultantIdentity getLatestPose() {
    return this.atomicEstimatedCameraTransform.get();
  }
}
