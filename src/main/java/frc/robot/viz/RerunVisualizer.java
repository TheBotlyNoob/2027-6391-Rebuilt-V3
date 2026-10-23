package frc.robot.viz;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LoggedTunableNumber;
import org.teamvan.moneyshift.NtRecorder;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;

/**
 * Publishes field, robot, and plot data over NetworkTables for the Rerun bridge.
 *
 * <p>moneyshift writes plain NetworkTables topics under {@code /Rerun}; the Rust bridge
 * ({@code rerun-importer-firstrun --nt}) turns those topics into Rerun chunks in real time. There
 * is no recording file and no blueprint: open the bridge instead of a {@code .rrd}. AdvantageKit
 * stays the source of truth — this layer only mirrors data that is already available on the robot
 * and never feeds state back into the robot.
 *
 * <p>Enabled by {@code /Tuning/Visualization/RerunEnabled}: on by default in simulation, off in
 * REAL and REPLAY. The system property {@code -Dfrc.robot.viz=0|1} overrides it (used by the
 * regression tests). Disabling stops publishing; the topics stay on the server until it restarts.
 *
 * <p>WASM builds substitute a no-op version of this class, so the NetworkTables JNI stack never
 * reaches the native image.
 */
public final class RerunVisualizer {

  private static final LoggedTunableNumber enabledTunable =
      new LoggedTunableNumber(
          "Visualization/RerunEnabled", Constants.currentMode == Mode.SIM ? 1.0 : 0.0);

  /**
   * Publishing rate. Every logged entity becomes one NetworkTables update, so the traffic scales
   * with rate times entity count.
   */
  private static final LoggedTunableNumber logHz =
      new LoggedTunableNumber("Visualization/RerunHz", 10.0);

  private final VizInputs inputs;
  private final NtRecorder recorder = NtRecorder.getInstance();
  private final RobotScene robotScene = new RobotScene();
  private final DataScene dataScene = new DataScene();

  private boolean staticLogged;
  private long loop;
  private double lastLogTime = Double.NEGATIVE_INFINITY;
  private boolean closed;
  private boolean failed;
  private String lastAutoName = "";
  private boolean lastAutoRed;

  public RerunVisualizer(VizInputs inputs) {
    this.inputs = inputs;
  }

  /** Captures one frame. Cheap and safe to call every loop; does nothing while disabled. */
  public void periodic() {
    if (closed || !shouldRun()) {
      return;
    }
    try {
      refreshAutoPath();
      double now = Timer.getTimestamp();
      double period = 1.0 / Math.max(1.0, logHz.get());
      if (now - lastLogTime < period) {
        return;
      }
      lastLogTime = now;
      logFrame(captureFrame());
    } catch (RuntimeException e) {
      fail("publishing a frame", e);
    }
  }

  /** Flushes pending NetworkTables updates. Idempotent. */
  public void close() {
    if (closed) {
      return;
    }
    closed = true;
    try {
      recorder.flush();
    } catch (RuntimeException e) {
      fail("flushing", e);
    }
  }

  private boolean shouldRun() {
    if (failed) {
      return false;
    }
    // -Dfrc.robot.viz=0|1 overrides the tunable (used by the regression tests).
    String override = System.getProperty("frc.robot.viz");
    if (override != null && !override.isBlank()) {
      String value = override.trim();
      return !(value.equals("0") || value.equalsIgnoreCase("false") || value.equalsIgnoreCase("off"));
    }
    return enabledTunable.get() > 0.5;
  }

  /** Re-logs the auto path overlay only when the selection or alliance changes. */
  private void refreshAutoPath() {
    String autoName = inputs.selectedAutoName();
    boolean red = inputs.redAlliance();
    if (!autoName.equals(lastAutoName) || red != lastAutoRed) {
      lastAutoName = autoName;
      lastAutoRed = red;
      FieldScene.logAutoPath(recorder, inputs.selectedAutoPath());
    }
  }

  private VizFrame captureFrame() {
    Drive drive = inputs.drive();
    Intake intake = inputs.intake();
    Indexer indexer = inputs.indexer();
    Shooter shooter = inputs.shooter();
    Vision vision = inputs.vision();
    String autoName = inputs.selectedAutoName();
    boolean red = inputs.redAlliance();
    double robotTime = Timer.getTimestamp();

    return new VizFrame(
        loop,
        robotTime,
        drive.getPose(),
        drive.getSimulationPose(),
        drive.getFieldRelativeSpeeds(),
        drive.getPitch(),
        intake.isDeployed(),
        intake.getGoal().name(),
        intake.getRollerVelocityRPM(),
        indexer.getGoal().name(),
        shooter.getGoal().name(),
        shooter.getAverageVelocityRPM(),
        shooter.getCommandedRPM(),
        shooter.getHoodAngleDeg(),
        shooter.isAtSetpoint(),
        shooter.getAimTarget(),
        vision.getVisibleTagIds(),
        vision.hasTagsInView(),
        inputs.driveMode(),
        autoName,
        red,
        inputs.hubActive(),
        inputs.hubActivatingSoon(),
        RobotState.isAutonomousEnabled(),
        RobotState.isTeleopEnabled(),
        RobotState.isDisabled());
  }

  /** Publishes one frame. Static field geometry is published on the first frame. */
  void logFrame(VizFrame frame) {
    if (!staticLogged) {
      FieldScene.logStatic(recorder);
      staticLogged = true;
    }
    robotScene.log(recorder, frame);
    dataScene.log(recorder, frame);
    loop = frame.loop() + 1;
  }

  private void fail(String action, Throwable cause) {
    if (failed) {
      return;
    }
    failed = true;
    System.err.println("[viz] disabled after failure while " + action + ": " + cause);
    cause.printStackTrace();
  }
}
