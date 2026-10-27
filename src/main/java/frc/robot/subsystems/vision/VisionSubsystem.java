package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.SheriffState;
import frc.robot.subsystems.vision.VisionSubsystemConstants.AprilTagCameraConfig;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystem extends SubsystemBase {
  public static VisionSystemSim visionSim;

  public static record AprilTagCamera(
      AprilTagCameraIO io,
      AprilTagIOInputsAutoLogged inputs,
      VisionSubsystemConstants.VisionSource source) {}

  public List<AprilTagCamera> aprilTagCameras = new ArrayList<>();

  public VisionSubsystem() {
    for (AprilTagCameraConfig config : VisionSubsystemConstants.CAMERA_CONFIGS) {
      AprilTagCameraIO io;

      switch (RobotConstants.currentMode) {
        case REAL:
          if (config.servoPort() >= 0) {
            io = new TurretedAprilTagCameraIOPhoton(config.source(), config.servoPort());
          } else {
            io = new AprilTagCameraIOPhoton(config.source());
          }

          break;
        case SIM:
          io = new AprilTagCameraIOPhotonSim(config.source(), config.simConfig());

          break;
        case REPLAY:
        default:
          io = new AprilTagCameraIO() {};
          break;
      }

      aprilTagCameras.add(
          new AprilTagCamera(io, new AprilTagIOInputsAutoLogged(), config.source()));
    }
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      VisionSubsystemConstants.aprilTagSim.get().update(SheriffState.robotPose);
    }
    for (AprilTagCamera camera : aprilTagCameras) {
      camera.io.updateInputs(camera.inputs);
      Logger.processInputs("VisionSubsystem/" + camera.source.name(), camera.inputs);
    }
  }
}
