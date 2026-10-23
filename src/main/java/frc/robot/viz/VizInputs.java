package frc.robot.viz;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;

/**
 * Read-only view of the robot that the Rerun visualization samples each loop.
 *
 * <p>{@link frc.robot.RobotContainer} implements this so the visualization layer never reaches
 * into subsystem internals. It intentionally contains no Rerun types: WASM builds substitute a
 * no-op {@link RerunVisualizer} while still compiling this interface.
 */
public interface VizInputs {
  Drive drive();

  Intake intake();

  Indexer indexer();

  Shooter shooter();

  Vision vision();

  boolean redAlliance();

  String driveMode();

  String selectedAutoName();

  /** Pose samples of the selected autonomous routine, already mirrored/flipped for the alliance. */
  List<Pose2d> selectedAutoPath();

  boolean hubActive();

  boolean hubActivatingSoon();
}
