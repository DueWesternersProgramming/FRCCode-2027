package frc.robot.subsystems.lidar;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Velocity filter that sits between a path follower and the drivetrain.
 *
 * <p>Each call it picks the single most urgent obstacle on a collision course and adds a sideways
 * velocity (perpendicular to the direction of travel, away from the obstacle) just large enough to
 * clear it by the time we would reach it. Rotation is never modified. With no threat, the input is
 * returned untouched.
 */
public class ObstacleAvoidance {
  private final double robotRadius; // m: half the robot's footprint (use half-diagonal to be safe)
  private final double margin; // m: extra clearance
  private final double horizon; // s: ignore anything we wouldn't reach this soon
  private final double latMax; // m/s: cap on the sideways nudge
  private final double vMax; // m/s: cap on the final translation speed

  private double lastSide =
      1; // +1 = dodge to our left, -1 = to our right (used when box is dead ahead)

  // for logging
  private int threatCount = 0;

  public ObstacleAvoidance(
      double robotRadius, double margin, double horizon, double latMax, double vMax) {
    this.robotRadius = robotRadius;
    this.margin = margin;
    this.horizon = horizon;
    this.latMax = latMax;
    this.vMax = vMax;
  }

  public int getThreatCount() {
    return threatCount;
  }

  /**
   * @param robotRel the speeds the path follower wants (robot-relative)
   * @param pose current robot pose in the field
   * @param obstacles field-frame boxes (stale ones already removed)
   * @return speeds to actually drive (robot-relative)
   */
  public ChassisSpeeds filter(ChassisSpeeds robotRel, Pose2d pose, List<Obstacle> obstacles) {
    threatCount = 0;
    if (obstacles.isEmpty()) return robotRel;

    Rotation2d heading = pose.getRotation();
    ChassisSpeeds f = ChassisSpeeds.fromRobotRelativeSpeeds(robotRel, heading);
    Translation2d vDes = new Translation2d(f.vxMetersPerSecond, f.vyMetersPerSecond);

    List<Obstacle> threats = topThreats(obstacles, pose.getTranslation(), vDes, 1);
    threatCount = threats.size();
    if (threats.isEmpty()) return robotRel;

    Translation2d v = avoid(vDes, pose.getTranslation(), threats);
    if (v == vDes) return robotRel; // no nudge needed after all: avoid float round-trip

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        v.getX(), v.getY(), robotRel.omegaRadiansPerSecond, heading);
  }

  /**
   * Rank obstacles by urgency. An obstacle is a threat if, assuming constant velocities, our
   * closest approach within the horizon is inside its reach. Most urgent (soonest closest-approach)
   * first.
   */
  List<Obstacle> topThreats(List<Obstacle> obs, Translation2d pos, Translation2d v, int k) {
    record Scored(Obstacle o, double t) {}
    List<Scored> threats = new ArrayList<>();

    for (Obstacle o : obs) {
      Translation2d p = o.center().minus(pos); // where it is relative to us
      Translation2d w = o.vel().minus(v); // how that gap changes per second
      double ww = w.getX() * w.getX() + w.getY() * w.getY();
      double t =
          ww < 1e-6
              ? 0
              : MathUtil.clamp(-(p.getX() * w.getX() + p.getY() * w.getY()) / ww, 0, horizon);
      double gap = p.plus(w.times(t)).getNorm();
      double reach = Math.hypot(o.half().getX(), o.half().getY()) + robotRadius + margin;
      if (gap < reach) threats.add(new Scored(o, t));
    }

    threats.sort(Comparator.comparingDouble(Scored::t));
    List<Obstacle> out = new ArrayList<>();
    for (int i = 0; i < Math.min(k, threats.size()); i++) out.add(threats.get(i).o());
    return out;
  }

  /**
   * Add a perpendicular nudge away from each given obstacle. Returns the same object as vDes if
   * nothing was changed.
   */
  Translation2d avoid(Translation2d vDes, Translation2d pos, List<Obstacle> obstacles) {
    Translation2d v = vDes;
    boolean changed = false;

    for (Obstacle b : obstacles) {
      Translation2d vRel = v.minus(b.vel()); // our motion as the obstacle sees it
      double s = vRel.getNorm();
      if (s < 0.1) continue; // barely moving relative to it

      Translation2d d = vRel.div(s); // direction of travel
      Translation2d n = new Translation2d(-d.getY(), d.getX()); // left of travel
      Translation2d toC = b.center().minus(pos);
      double along = toC.getX() * d.getX() + toC.getY() * d.getY(); // how far ahead
      double lat = toC.getX() * n.getX() + toC.getY() * n.getY(); // + = box is on our left

      // Half-extent of the (field-axis-aligned) box measured along d and n
      double halfD = b.half().getX() * Math.abs(d.getX()) + b.half().getY() * Math.abs(d.getY());
      double halfN = b.half().getX() * Math.abs(n.getX()) + b.half().getY() * Math.abs(n.getY());
      double buffer = robotRadius + margin;
      double clearance = halfN + buffer; // sideways distance we need from the box center

      if (along + halfD < 0) continue; // box is entirely behind us
      if (Math.abs(lat) >= clearance) continue; // our line of travel already misses it

      double tReach = Math.max(along - halfD - robotRadius, 0.05) / s;
      if (tReach > horizon) continue; // too far away to matter yet

      // Dodge away from the box; if it's dead ahead, keep the previous choice.
      if (Math.abs(lat) > 0.05) lastSide = -Math.signum(lat);

      double shift = clearance - Math.abs(lat); // meters we still must move over
      double latSpeed = Math.min(shift / tReach, latMax);
      v = v.plus(n.times(lastSide * latSpeed));
      changed = true;
    }

    if (!changed) return vDes;
    double norm = v.getNorm();
    return norm > vMax ? v.times(vMax / norm) : v;
  }
}
