package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.SheriffState;
import frc.robot.subsystems.vision.VisionSubsystemConstants.VisionSource;
import frc.robot.utils.CowboyUtils;
import frc.robot.utils.TimestampedPose;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagCameraIOPhoton implements AprilTagCameraIO {
  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimator;

  public AprilTagCameraIOPhoton(VisionSource source) {
    photonCamera = new PhotonCamera(source.name());

    photonPoseEstimator =
        new PhotonPoseEstimator(
            CowboyUtils.aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            source.robotToCamera());
    photonPoseEstimator.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    inputs.mounting = photonPoseEstimator.getRobotToCameraTransform();
    inputs.connected = photonCamera.isConnected();
    inputs.targetIDs =
        photonCamera.getLatestResult().getTargets().stream()
            .map(PhotonTrackedTarget::getFiducialId)
            .mapToInt(Integer::intValue)
            .toArray();
    inputs.hasTargets = photonCamera.getLatestResult().hasTargets();

    List<PhotonPipelineResult> results =
        photonCamera.getAllUnreadResults(); // Call this only once!!

    if (!results.isEmpty()) {
      try {

        if (results.size() > 0) {
          for (PhotonPipelineResult result : results) {
            if (result.hasTargets()) {
              PhotonTrackedTarget target = result.getBestTarget();

              inputs.bestTargetArea = target.getArea();

              Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(result);
              estimatedRobotPose.ifPresent(
                  est -> {
                    Pose2d pose = estimatedRobotPose.get().estimatedPose.toPose2d();
                    inputs.pose = pose;
                    SheriffState.offerAprilTagCameraMeasurement(
                        new TimestampedPose(pose, estimatedRobotPose.get().timestampSeconds));
                  });
            }
          }
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  @Override
  public List<PhotonPipelineResult> getResult() {

    return photonCamera.getAllUnreadResults();
  }

  @Override
  public boolean isCameraConnected() {
    return photonCamera.isConnected();
  }

  @Override
  public PhotonTrackedTarget getBestTarget() {
    return getResult().get(0).getBestTarget();
  }

  @Override
  public double getBestTargetYaw() {
    return getBestTarget().getYaw();
  }

  @Override
  public void setCameraTransformOffset(Transform3d offset) {
    photonPoseEstimator.setRobotToCameraTransform(offset);
  }
}
