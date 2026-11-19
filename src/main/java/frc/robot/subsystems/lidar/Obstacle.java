package frc.robot.subsystems.lidar;

import edu.wpi.first.math.geometry.Translation2d;

public record Obstacle(Translation2d center, Translation2d half, Translation2d vel) {}
