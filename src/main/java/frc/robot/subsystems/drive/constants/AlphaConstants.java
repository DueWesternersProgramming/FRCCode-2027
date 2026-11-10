package frc.robot.subsystems.drive.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Alpha bot (REV NEO/Spark + SDS MK4c) settings. This is the Spark-side equivalent of
 * TunerConstants. Every value marked TODO is a placeholder: measure or tune it on the robot.
 *
 * <p>AI Generated File!!!
 */
public final class AlphaConstants extends DriveConstants {
  private AlphaConstants() {}

  /** Per-module hardware settings consumed by ModuleIOSpark. */
  public record SparkModuleConfig(
      int driveCanId,
      int turnCanId,
      Rotation2d zeroRotation, // Absolute encoder reading when the wheel points forward
      boolean driveInverted,
      boolean turnInverted,
      boolean turnEncoderInverted) {}

  // ---------------------------------------------------------------------------
  // Robot-level values (feed DrivetrainConfigs.ALPHA)
  // ---------------------------------------------------------------------------
  public static final double WHEEL_RADIUS_METERS =
      Units.inchesToMeters(2.0); // TODO verify (worn wheels are smaller)
  public static final double MAX_SPEED_METERS_PER_SEC = 4.4; // TODO measure free speed at 12 V
  public static final double ROBOT_MASS_KG = 50.0; // TODO
  public static final double ROBOT_MOI = 5.0; // TODO kg*m^2
  public static final double ODOMETRY_FREQUENCY_HZ = 100.0; // Spark odometry thread rate

  // Module positions relative to robot center, in order FL, FR, BL, BR
  private static final double TRACK_HALF_X = Units.inchesToMeters(10.0); // TODO forward/back
  private static final double TRACK_HALF_Y = Units.inchesToMeters(10.0); // TODO left/right
  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(TRACK_HALF_X, TRACK_HALF_Y), // FL
    new Translation2d(TRACK_HALF_X, -TRACK_HALF_Y), // FR
    new Translation2d(-TRACK_HALF_X, TRACK_HALF_Y), // BL
    new Translation2d(-TRACK_HALF_X, -TRACK_HALF_Y) // BR
  };

  // ---------------------------------------------------------------------------
  // MK4c gearing: L1 = 8.14, L2 = 6.75, L3 = 6.12 (steer is 12.8:1 on all)
  // ---------------------------------------------------------------------------
  public static final double DRIVE_GEAR_RATIO = 6.75;
  public static final double TURN_GEAR_RATIO = 12.8;

  // ---------------------------------------------------------------------------
  // Current limits (amps)
  // ---------------------------------------------------------------------------
  public static final int DRIVE_CURRENT_LIMIT = 40;
  public static final int TURN_CURRENT_LIMIT = 20;

  // ---------------------------------------------------------------------------
  // Encoder conversion factors (Spark reports in these units after conversion)
  // ---------------------------------------------------------------------------
  // Drive: motor rotations -> wheel radians
  public static final double DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / DRIVE_GEAR_RATIO;
  // Drive: motor RPM -> wheel rad/sec
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR =
      (2 * Math.PI) / 60.0 / DRIVE_GEAR_RATIO;
  // Turn: absolute encoder is on the module output shaft, so no gear ratio
  public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI; // rad
  public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // rad/sec

  // ---------------------------------------------------------------------------
  // Closed-loop gains (volts based, tune on the robot)
  // ---------------------------------------------------------------------------
  public static final double DRIVE_KP = 2.0; // TODO
  public static final double DRIVE_KD = 0.0;
  public static final double DRIVE_KS = 0.0; // TODO
  public static final double DRIVE_KV = 0.1; // TODO ~ 12 / (free speed in wheel rad/sec)

  public static final double TURN_KP = 2.0; // TODO
  public static final double TURN_KD = 0.0;
  public static final double TURN_PID_MIN_INPUT = 0.0; // rad
  public static final double TURN_PID_MAX_INPUT = 2 * Math.PI; // rad

  public static final double SPARK_ODOMETRY_FREQUENCY = 100.0; // Hz, Spark odometry thread rate

  // ---------------------------------------------------------------------------
  // Per-module hardware: CAN IDs, encoder zeros, inversions
  // ---------------------------------------------------------------------------
  public static final SparkModuleConfig FL =
      new SparkModuleConfig(1, 2, Rotation2d.fromRotations(0.0), false, false, true);

  public static final SparkModuleConfig FR =
      new SparkModuleConfig(3, 4, Rotation2d.fromRotations(0.0), false, false, true);

  public static final SparkModuleConfig BL =
      new SparkModuleConfig(5, 6, Rotation2d.fromRotations(0.0), false, false, true);

  public static final SparkModuleConfig BR =
      new SparkModuleConfig(7, 8, Rotation2d.fromRotations(0.0), false, false, true);
}
