package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotConstants;

public class CowboyUtils {

  public static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        ? (DriverStation.getAlliance().get() == Alliance.Red)
        : (false);
  }

  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().isPresent()
        ? (DriverStation.getAlliance().get() == Alliance.Blue)
        : (false);
  }

  public static boolean isFMSAttached() {
    return DriverStation.isFMSAttached();
  }

  /**
   * Returns true if the robot is running in simulation and is connected to the FMS. This is used to
   * determine if the robot is running in a competition simulation.
   */
  public static boolean isCompBotMode() {
    return RobotConstants.robot.equals(RobotConstants.Robot.COMP);
  }

  public static boolean isSim() {
    return RobotBase.isSimulation();
  }

  /**
   * @see https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
   */
  public static double getParallelError(Pose2d origin, Pose2d target) {
    Translation2d originToTarget = origin.minus(target).getTranslation();
    Rotation2d angleBetween = originToTarget.getAngle();
    double parallelError = originToTarget.getNorm() * angleBetween.getSin();

    return parallelError;

    // return origin.minus(target).getY();
  }

  /**
   * @see https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
   */
  public static double getPerpendicularError(Pose2d origin, Pose2d target) {
    Translation2d originToTarget = origin.minus(target).getTranslation();
    Rotation2d angleBetween = originToTarget.getAngle();
    double perpendicularError = originToTarget.getNorm() * angleBetween.getCos();

    return perpendicularError;

    // return -origin.minus(target).getX();
  }

  public static Rotation2d getAngleToPose(Pose2d pose1, Pose2d pose2) {
    double dx = pose1.getX() - pose2.getX();
    double dy = pose1.getY() - pose2.getY();

    return new Rotation2d(Units.degreesToRadians(Math.atan2(dy, dx)));
  }
}
