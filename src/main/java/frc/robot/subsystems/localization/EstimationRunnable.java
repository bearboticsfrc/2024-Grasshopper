package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.VisionConstants;
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

    photonResults.targets.removeIf(
        t ->
            t.getPoseAmbiguity() > VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD
                || isTargetTooFarAway(t));

    if (!photonResults.hasTargets()) return;

    atomicEstimatedCameraTransform.set(getTransform(photonResults));
  }

  /**
   * Determines whether a detected target is too far away based on a predefined culling distance.
   *
   * @param target The PhotonTrackedTarget to evaluate.
   * @return True if the target is beyond the culling distance, false otherwise.
   */
  private boolean isTargetTooFarAway(PhotonTrackedTarget target) {
    double pitch = target.getPitch();
    if (!layout.getTagPose(target.getFiducialId()).isPresent()) {
      return false;
    }
    return ((layout.getTagPose(target.getFiducialId()).get().getZ()) / Math.atan(Math.toRadians(pitch)))
        < VisionConstants.APRILTAG_CULL_DISTANCE;
  }

  public CameraPoseResultantIdentity getTransform(PhotonPipelineResult result) {
    int count = 0;
    double dist = 0;
    double y = 0;
    double x = 0;

    double yaw = 0;
    for (PhotonTrackedTarget i : (result.targets)) {
      Pose3d tagPose = layout.getTagPose(i.getFiducialId()).get();
      double individualZ = tagPose.getZ();

      count += 1;
      double individualDist = individualZ / Math.tan(Math.toRadians(i.getPitch()));
      dist += (individualDist);
      yaw += tagPose.getRotation().getZ() + i.getYaw();
      y += tagPose.getX() + (individualDist / Math.asin(Math.toRadians(i.getYaw())));
      x += tagPose.getY() + (individualDist / Math.cos( Math.toRadians(i.getYaw())));
    
    }
    dist /= count;
    System.out.println("distance"+dist);
    y /= count;
    x /= count;
    yaw /= count;
    double time = result.getTimestampSeconds();
 
    return new CameraPoseResultantIdentity(dist, y, x, yaw, time);
  }

  public CameraPoseResultantIdentity getLatestPose() {
    return this.atomicEstimatedCameraTransform.get();
  }
}
