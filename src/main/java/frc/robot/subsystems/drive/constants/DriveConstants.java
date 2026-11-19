package frc.robot.subsystems.drive.constants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CowboyUtils;
import java.util.Arrays;

public class DriveConstants {
  public class DriveCommandsConstants {
    public static final double DEADBAND = 0.1;
    public static final double ANGLE_KP = 5.0;
    public static final double ANGLE_KD = 0.4;
    public static final double ANGLE_MAX_VELOCITY = 8.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    public static final double MAX_DRIVE_PERCENTAGE = 1;
  }

  public static final double ODOMETRY_FREQUENCY =
      TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;

  public static final double DRIVE_BASE_RADIUS =
      Arrays.stream(getModuleTranslations()).mapToDouble(Translation2d::getNorm).max().orElse(0.0);

  /** Returns an array of module translations (FL, FR, BL, BR) for the active robot. */
  public static Translation2d[] getModuleTranslations() {
    if (CowboyUtils.isCompBotMode()) {
      return new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
      };
    }
    // Alphabot
    return AlphaConstants.MODULE_TRANSLATIONS;
  }
}
