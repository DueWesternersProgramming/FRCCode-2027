package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.automation.DriveToPoseCommand;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.CowboyUtils;
import java.io.IOException;

public class BLine {
  public static Command BLineTrajectory(Drive drive, String name, boolean mirrorVertically) {
    Path path = new Path(name);

    if (mirrorVertically) {
      path.mirror();
    }

    Pose2d startPose =
        CowboyUtils.isRedAlliance()
            ? FlippingUtil.flipFieldPose(path.getStartPose())
            : path.getStartPose();

    return Commands.either(
            drive.getBLineBuilder().build(path),
            Commands.sequence(
                new DriveToPoseCommand(drive, () -> startPose),
                drive.getBLineBuilder().build(path)),
            () ->
                drive.getPose().getTranslation().getDistance(startPose.getTranslation()) < 0.10
                    && Math.abs(
                            drive
                                .getPose()
                                .getRotation()
                                .minus(startPose.getRotation())
                                .getDegrees())
                        < 5.0)
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.print(name + ": end"));
  }

  public static Command BLineTrajectory(Drive drive, Path path, boolean mirrorVertically) {
    if (mirrorVertically) {
      path.mirror();
    }

    Pose2d startPose =
        CowboyUtils.isRedAlliance()
            ? FlippingUtil.flipFieldPose(path.getStartPose())
            : path.getStartPose();

    System.out.println("Start Pose: " + startPose);

    return Commands.either(
        drive.getBLineBuilder().build(path),
        Commands.sequence(
            new DriveToPoseCommand(drive, () -> startPose), drive.getBLineBuilder().build(path)),
        () ->
            drive.getPose().getTranslation().getDistance(startPose.getTranslation()) < 0.10
                && Math.abs(
                        drive.getPose().getRotation().minus(startPose.getRotation()).getDegrees())
                    < 5.0);
  }

  public static Path getPath(String name, boolean mirrorVertically) {
    Path path = new Path(name);

    if (mirrorVertically) {
      path.mirror();
    }

    return path;
  }

  public static String getPathJson(String name) throws IOException {
    try {
      java.nio.file.Path file =
          java.nio.file.Paths.get(
              Filesystem.getDeployDirectory().getAbsolutePath(), "autos", "paths", name + ".json");

      return java.nio.file.Files.readString(file);

    } catch (IOException e) {
      DriverStation.reportError("Failed to load B-Line path JSON: " + e, true);

      java.nio.file.Path file =
          java.nio.file.Paths.get(
              Filesystem.getDeployDirectory().getAbsolutePath(),
              "autos",
              "paths",
              "nothingpath" + ".json");

      return java.nio.file.Files.readString(file);
    }
  }
}
