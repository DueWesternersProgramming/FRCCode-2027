package frc.robot.subsystems.lidar;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import java.util.function.Supplier;

public interface LidarIO {
  default List<Pose2d> getTrackedTargets() {
    return List.of();
  }

  default void updateObstacles(
      SwerveDrivePoseEstimator poseEstimator,
      Supplier<ChassisSpeeds> getFieldRelativeChassisSpeeds) {}
}
