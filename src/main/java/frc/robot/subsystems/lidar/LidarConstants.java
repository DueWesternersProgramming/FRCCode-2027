package frc.robot.subsystems.lidar;

import edu.wpi.first.math.geometry.Translation2d;

public class LidarConstants {
  public static final double AVOID_ROBOT_RADIUS_M =
      0.5; // half-diagonal of OUR bumper footprint, to be safe
  public static final double AVOID_MARGIN_M = 0.2; // extra clearance
  public static final double AVOID_HORIZON_S = 1.5; // ignore obstacles we wouldn't reach this soon
  public static final double AVOID_LATERAL_MAX_MPS = 3.0; // cap on the sideways nudge

  public static final Translation2d OBSTACLE_HALF =
      new Translation2d(0.64, 0.64); // covers a 0.9 m bumper box at
  // any angle
  public static final double FIELD_LENGTH_M = 17.55; // SET to this year's field
  public static final double FIELD_WIDTH_M = 8.05; // SET to this year's field
  public static final double FIELD_EDGE_MARGIN_M =
      0.3; // detections this close to / beyond a wall are dropped
}
