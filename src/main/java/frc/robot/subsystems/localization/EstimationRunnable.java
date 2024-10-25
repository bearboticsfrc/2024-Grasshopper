package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
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
    double sumY = 0;
    double sumX = 0;
    double sumYaw = 0;

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

      // double nueralY = i.getBestCameraToTarget().getX()-tagPose.getX();
      // double nueralX = i.getBestCameraToTarget().getY()-tagPose.getY();
      double nueralY = i.getBestCameraToTarget().getX();
      double nueralX = i.getBestCameraToTarget().getY();
      double indYaw = i.getYaw();

      // CoordinateTransform lambdaCoordinateTransform = new CoordinateTransform(nueralY, nueralX,
      // true);

      CoordinateTransform trigTransform = new CoordinateTransform(individualDist, indYaw, false);
      CoordinateTransform nueralTransform = new CoordinateTransform(nueralY, nueralX, true);

      double avgR = (trigTransform.getR() + nueralTransform.getR()) / 2;
      double avgTheta = (trigTransform.getTheta() + nueralTransform.getTheta()) / 2;
      CoordinateTransform avgTransform = new CoordinateTransform(avgR, avgTheta, false);
      sumY += avgTransform.getY() + tagPose.getY();
      sumX += avgTransform.getX() + tagPose.getX();
      sumYaw += indYaw;
    }
    sumYaw /= count;
    sumY /= (count);
    sumX /= (count);
    double time = result.getTimestampSeconds();

    return new CameraPoseResultantIdentity(new CoordinateTransform(sumY, sumX, true), time, sumYaw);
  }

  public CameraPoseResultantIdentity getLatestPose() {
    return this.atomicEstimatedCameraTransform.get();
  }
}
