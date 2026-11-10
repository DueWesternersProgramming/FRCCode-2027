// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.HexFormat;
import java.util.Set;

public final class RobotConstants {
  // Change the value of "simMode" to switch between "sim" (physics sim) and
  // "replay" (log replay
  // from a file).
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Robot {
    COMP,
    ALPHA
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class ScoringConstants {}

  private static final Set<String> ALPHA_MACS = Set.of("00:80:2F:XX:XX:XX");
  private static final Set<String> COMP_MACS = Set.of("00:80:2F:YY:YY:YY");

  public static final Robot robot = detectRobot();

  private static Robot detectRobot() {
    if (!RobotBase.isReal()) return Robot.COMP; // Default to COMP for simulation and replay
    try {
      for (var nic : Collections.list(NetworkInterface.getNetworkInterfaces())) {
        byte[] mac = nic.getHardwareAddress();
        if (mac == null) continue;
        String s = HexFormat.ofDelimiter(":").withUpperCase().formatHex(mac);
        if (ALPHA_MACS.contains(s)) return Robot.ALPHA;
        if (COMP_MACS.contains(s)) return Robot.COMP;
      }
    } catch (SocketException e) {
      e.printStackTrace();
    }
    DriverStation.reportError("Unknown MAC, defaulting to COMP", false);
    return Robot.COMP;
  }
}
