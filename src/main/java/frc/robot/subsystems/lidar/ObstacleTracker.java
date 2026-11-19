package frc.robot.subsystems.lidar;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;

public class ObstacleTracker {
  private static final double ALPHA = 0.6;
  private static final double BETA = 0.25;
  private static final double GATE_M = 0.5;
  private static final double V_MAX = 5.0;
  private static final double STILL = 0.3;
  private static final double MAX_AGE = 0.3;
  private static final int CONFIRM_HITS = 2;

  // --- ego-motion rejection ---
  private static final double EGO_MIN_SPEED =
      0.5; // m/s: robot must be moving for the check to apply
  private static final double EGO_MIN_COS =
      0.7; // |cos(angle)| between track vel and robot vel (~45 deg)
  private static final double EGO_RATIO_MIN =
      0.3; // track speed / robot speed range that counts as "similar"
  private static final double EGO_RATIO_MAX = 2.0;
  private static final double MAX_EXTRAPOLATE = 0.15; // s: never project a track further than this

  private static class Track {
    double x, y, vx, vy, lastT;
    int hits = 1;
  }

  private final List<Track> tracks = new ArrayList<>();
  private int lastEgoRejects = 0;

  /**
   * True if the track's velocity looks like it is just a reflection of our own motion (aligned with
   * it, either direction, and a similar magnitude).
   */
  private static boolean looksLikeEgoMotion(double vx, double vy, ChassisSpeeds robotVel) {
    double rs = Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);
    double ts = Math.hypot(vx, vy);
    if (rs < EGO_MIN_SPEED || ts < STILL)
      return false; // robot parked, or track already counts as still
    double cos = (vx * robotVel.vxMetersPerSecond + vy * robotVel.vyMetersPerSecond) / (rs * ts);
    double ratio = ts / rs;
    return Math.abs(cos) > EGO_MIN_COS && ratio > EGO_RATIO_MIN && ratio < EGO_RATIO_MAX;
  }

  /**
   * Feed one scan's detections (field frame) captured at time {@code t}. {@code robotVel} is our
   * field-relative velocity (m/s).
   */
  public void update(List<Translation2d> dets, double t, ChassisSpeeds robotVel) {
    boolean[] used = new boolean[dets.size()];
    lastEgoRejects = 0;

    for (Track tr : tracks) {
      double dt = t - tr.lastT;
      if (dt <= 1e-3) continue;

      double px = tr.x + tr.vx * dt;
      double py = tr.y + tr.vy * dt;
      double gate = GATE_M + 0.5 * V_MAX * dt;

      int best = -1;
      double bestD = gate;
      for (int i = 0; i < dets.size(); i++) {
        if (used[i]) continue;
        double d = Math.hypot(dets.get(i).getX() - px, dets.get(i).getY() - py);
        if (d < bestD) {
          bestD = d;
          best = i;
        }
      }
      if (best < 0) continue;

      used[best] = true;
      double rx = dets.get(best).getX() - px;
      double ry = dets.get(best).getY() - py;
      tr.x = px + ALPHA * rx;
      tr.y = py + ALPHA * ry;
      tr.vx += BETA * rx / dt;
      tr.vy += BETA * ry / dt;

      double sp = Math.hypot(tr.vx, tr.vy);
      if (sp > V_MAX) {
        tr.vx *= V_MAX / sp;
        tr.vy *= V_MAX / sp;
      }

      // Velocity that mirrors our own motion is an artifact: kill it in the
      // filter state so it can't feed back into the next prediction.
      if (looksLikeEgoMotion(tr.vx, tr.vy, robotVel)) {
        tr.vx = 0;
        tr.vy = 0;
        lastEgoRejects++;
      }

      tr.lastT = t;
      tr.hits++;
    }

    for (int i = 0; i < dets.size(); i++) {
      if (used[i]) continue;
      Track tr = new Track();
      tr.x = dets.get(i).getX();
      tr.y = dets.get(i).getY();
      tr.lastT = t;
      tracks.add(tr);
    }

    tracks.removeIf(tr -> t - tr.lastT > MAX_AGE);
  }

  /** Confirmed, fresh tracks as boxes, projected forward (briefly) to {@code now}. */
  public List<Obstacle> getObstacles(double now, Translation2d half, ChassisSpeeds robotVel) {
    List<Obstacle> out = new ArrayList<>();
    for (Track tr : tracks) {
      double age = now - tr.lastT;
      if (tr.hits < CONFIRM_HITS || age > MAX_AGE) continue;

      boolean still =
          Math.hypot(tr.vx, tr.vy) < STILL || looksLikeEgoMotion(tr.vx, tr.vy, robotVel);
      Translation2d vel = still ? new Translation2d() : new Translation2d(tr.vx, tr.vy);

      double dt = Math.min(age, MAX_EXTRAPOLATE); // cap how far a velocity can carry it
      out.add(
          new Obstacle(
              new Translation2d(tr.x + vel.getX() * dt, tr.y + vel.getY() * dt), half, vel));
    }
    return out;
  }

  public int lastEgoRejects() {
    return lastEgoRejects;
  }

  public void clear() {
    tracks.clear();
  }
}
