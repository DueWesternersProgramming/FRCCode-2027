package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.TimestampedPose;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import org.littletonrobotics.junction.AutoLogOutput;

public class SheriffState {
  public static boolean isObstacleAvoidanceEnabled = false;
  public static boolean isAntiTippingEnabled = false;
  public static boolean isScoringAutomationActive = false;
  public static boolean xLocked = false;
  public static Pose2d robotPose = new Pose2d();
  @AutoLogOutput public static boolean isQuestNavPoseReset = false;

  @AutoLogOutput
  private static final Queue<TimestampedPose> questMeasurements = new LinkedBlockingQueue<>(20);

  private static final Queue<TimestampedPose> aprilTagCameraMeasurements =
      new LinkedBlockingQueue<>(20);

  public static Queue<TimestampedPose> getQuestMeasurments() {
    return questMeasurements;
  }

  public static void offerQuestMeasurement(TimestampedPose observation) {
    questMeasurements.offer(observation);
  }

  public static Queue<TimestampedPose> getAprilTagCameraMeasurments() {
    return aprilTagCameraMeasurements;
  }

  public static void offerAprilTagCameraMeasurement(TimestampedPose observation) {
    aprilTagCameraMeasurements.offer(observation);
  }
}
