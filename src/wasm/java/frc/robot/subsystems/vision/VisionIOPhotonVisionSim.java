package frc.robot.subsystems.vision;

import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.telemetry.Telemetry;

/** WASM deliberately has no camera observations; desktop PhotonVision simulation is unchanged. */
public final class VisionIOPhotonVisionSim implements VisionIO, AutoCloseable {
  private static final TargetObservation NO_TARGET =
      new TargetObservation(Rotation2d.ZERO, Rotation2d.ZERO);
  private static final PoseObservation[] NO_POSES = new PoseObservation[0];
  private static final int[] NO_TAGS = new int[0];

  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    Telemetry.log("Vision/Simulation/Available", false);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = false;
    inputs.latestTargetObservation = NO_TARGET;
    inputs.poseObservations = NO_POSES;
    inputs.tagIds = NO_TAGS;
  }

  @Override
  public void close() {}
}
