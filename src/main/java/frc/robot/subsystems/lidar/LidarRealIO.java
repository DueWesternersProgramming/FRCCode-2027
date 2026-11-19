package frc.robot.subsystems.lidar;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.subsystems.drive.constants.AlphaConstants;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.utils.CowboyUtils;
import java.util.List;

public class LidarRealIO implements LidarIO {

  private List<Obstacle> obstacles = List.of();
  private final DoubleArraySubscriber lidarSub;
  private final ObstacleTracker obstacleTracker = new ObstacleTracker();
  private final ObstacleAvoidance avoidanceFilter =
      new ObstacleAvoidance(
          LidarConstants.AVOID_ROBOT_RADIUS_M,
          LidarConstants.AVOID_MARGIN_M,
          LidarConstants.AVOID_HORIZON_S,
          LidarConstants.AVOID_LATERAL_MAX_MPS,
          CowboyUtils.isCompBotMode()
              ? TunerConstants.kSpeedAt12Volts.magnitude()
              : AlphaConstants.MAX_SPEED_METERS_PER_SEC);

  public LidarRealIO() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("LidarProcessor");

    lidarSub =
        table
            .getDoubleArrayTopic("poses")
            .subscribe(
                new double[0], PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
  }

  @Override
  public void updateObstacles() {
    // for (TimestampedDoubleArray scan : lidarSub.readQueue()) {

    //   double currentTimeStamp = scan.timestamp / 1e6; // convert to seconds
    //   double scanTimestamp = scan.value[0];

    //   // System.out.println("TimestampedDoubleArray TS: " + currentTimeStamp + "\t
    //   // Scan Time: " + scanTimestamp
    //   // + "\t Delta: " + (currentTimeStamp - scanTimestamp));

    //   Pose2d robotAtScan = hybridOdometry.sampleAt(scanTimestamp).orElse(getPose());

    //   List<Translation2d> detections = new ArrayList<>();

    //   int raw = 0;
    //   // Filter out any detections that are outside the field boundaries
    //   for (int i = 1; i + 2 < scan.value.length; i += 3) {
    //     raw++;
    //     Translation2d local = new Translation2d(scan.value[i], scan.value[i + 1]);
    //     Translation2d field =
    //         robotAtScan.getTranslation().plus(local.rotateBy(robotAtScan.getRotation()));
    //     boolean onField =
    //         field.getX() > LidarConstants.FIELD_EDGE_MARGIN_M
    //             && field.getX() < LidarConstants.FIELD_LENGTH_M -
    // LidarConstants.FIELD_EDGE_MARGIN_M
    //             && field.getY() > LidarConstants.FIELD_EDGE_MARGIN_M
    //             && field.getY() < LidarConstants.FIELD_WIDTH_M -
    // LidarConstants.FIELD_EDGE_MARGIN_M;
    //     if (onField) detections.add(field);
    //   }
    //   obstacleTracker.update(detections, scanTimestamp, getFieldRelativeChassisSpeeds());

    //   Logger.recordOutput("Avoidance/RawDetections", raw);
    //   Logger.recordOutput("Avoidance/DetectionsKept", detections.size());
    //   Logger.recordOutput("Avoidance/ScanAge", currentTimeStamp - scanTimestamp);
    // }
    // obstacles =
    //     obstacleTracker.getObstacles(
    //         WPIUtilJNI.now() * 1e-6, OBSTACLE_HALF, getFieldRelativeChassisSpeeds());
    // Logger.recordOutput("Avoidance/ObstacleCount", obstacles.size());
    // Logger.recordOutput(
    //     "Avoidance/Obstacles",
    //     obstacles.stream()
    //         .map(o -> new Pose2d(o.center(), new Rotation2d()))
    //         .toArray(Pose2d[]::new));
  }
}
