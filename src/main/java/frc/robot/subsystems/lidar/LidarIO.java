package frc.robot.subsystems.lidar;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public interface LidarIO {
  default List<Pose2d> getTrackedTargets() {
    return List.of();
  }

  default void updateObstacles() {}
}
