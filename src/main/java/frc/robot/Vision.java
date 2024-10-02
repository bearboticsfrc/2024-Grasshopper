/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import static frc.robot.constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORM;
import static frc.robot.constants.VisionConstants.TAG_LAYOUT;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;

  private double lastEstTimestamp;

  public Vision() {
    camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

    photonEstimator =
        new PhotonPoseEstimator(
            TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            ROBOT_TO_CAMERA_TRANSFORM);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /**
   * Get the latest photon result from the camera
   *
   * @return The result
   */
  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();

    if (Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5) {
      lastEstTimestamp = latestTimestamp;
    }

    return visionEst;
  }

  /**
   * Returns the standard deviations for the estimated pose, used with the SwerveDrivePoseEstimator.
   * This applies when targets are visible.
   *
   * @param estimatedPose The pose for which to estimate the standard deviations.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    var targets = getLatestResult().getTargets();

    int numTags = 0;
    double avgDist = 0;

    // Calculate the number of visible tags and average distance
    for (var target : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(target.getFiducialId());

      if (tagPose.isEmpty()) {
        continue; // No pose
      }

      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0) {
      return VisionConstants.SINGLE_TAG_STD_DEVS; // No tags visible, return default std devs
    } else {
      avgDist /= numTags; // Compute average distance
    }

    // Use lower std devs if multiple targets are visible
    if (numTags > 1) {
      estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
    }

    // Increase std devs if far from a single target
    if (numTags == 1 && avgDist > 4) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    return estStdDevs;
  }
}
