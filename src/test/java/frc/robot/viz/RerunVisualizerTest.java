package frc.robot.viz;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.teamvan.moneyshift.NtRecorder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * Exercises the publishing path end to end: field geometry, robot footprint, plots, state panel,
 * events, and the 2D-only scope of the NetworkTables {@code /Rerun} protocol.
 */
class RerunVisualizerTest {
  private NetworkTableInstance instance;

  @BeforeEach
  void setUp() {
    instance = NetworkTableInstance.create();
    NtRecorder.setInstance(instance);
  }

  @AfterEach
  void tearDown() {
    System.clearProperty("frc.robot.viz");
    NtRecorder.setInstance(null);
    instance.close();
  }

  @Test
  void publishesFieldRobotAndPlotsOverNetworkTables() {
    RerunVisualizer visualizer = new RerunVisualizer(new UnusedInputs());

    visualizer.logFrame(frame(0));
    visualizer.logFrame(frame(1));
    visualizer.close();

    // Static field geometry is published once and flagged static.
    assertTrue(bool("/Rerun/field/boundary/.rerun/static"));
    assertEquals(
        "rerun.archetypes.LineStrips2D",
        string("/Rerun/field/boundary/.rerun/archetype"));
    assertEquals(1L, integer("/Rerun/field/boundary/.rerun/layout"));
    assertTrue(topicsUnder("/Rerun/field/tags").length > 0, "april tags are part of the field");

    // The robot footprint is dynamic 2D geometry that follows the pose.
    assertEquals(
        "rerun.archetypes.LineStrips2D", string("/Rerun/field/robot/.rerun/archetype"));
    assertEquals(0, topicsUnder("/Rerun/field/robot/.rerun/static").length);
    assertArrayEquals(new long[] {5}, integers("/Rerun/field/robot/strips/.count"));
    assertTrue(topicsUnder("/Rerun/field/robot/heading").length > 0);

    // The aim target is a 2D point from the current frame.
    assertArrayEquals(new double[] {4.0}, doubles("/Rerun/field/aim_target/positions/0"));
    assertArrayEquals(new double[] {4.0}, doubles("/Rerun/field/aim_target/positions/1"));

    // Observed tags are drawn on the field plane, not as a 3D point cloud.
    assertEquals(
        "rerun.archetypes.Points2D", string("/Rerun/field/tags/observed/.rerun/archetype"));

    // One entity per series, because the protocol carries one archetype per update.
    assertEquals("rerun.archetypes.Scalars", string("/Rerun/plots/ShooterRPM/measured/.rerun/archetype"));
    assertArrayEquals(new double[] {2950.0}, doubles("/Rerun/plots/ShooterRPM/measured/scalars"));
    assertArrayEquals(new double[] {3000.0}, doubles("/Rerun/plots/ShooterRPM/setpoint/scalars"));
    assertArrayEquals(new double[] {31.5}, doubles("/Rerun/plots/ShooterHoodDeg/scalars"));
    assertEquals("rerun.archetypes.Scalars", string("/Rerun/plots/DriveSpeed/.rerun/archetype"));

    // The state panel and event log carry human-readable text.
    String state = strings("/Rerun/logs/state/text")[0];
    assertTrue(state.contains("TELEOP"), state);
    assertTrue(state.contains("2950 / 3000 RPM"), state);
    assertEquals("mode -> TELEOP", strings("/Rerun/logs/events/text")[0]);
  }

  @Test
  void publishesOnly2dGeometry() {
    RerunVisualizer visualizer = new RerunVisualizer(new UnusedInputs());
    visualizer.logFrame(frame(0));

    // The old 3D scene root is gone: nothing is published under /Rerun/world.
    assertEquals(0, topicsUnder("/Rerun/world").length);
    assertEquals(0, topicsUnder("/Rerun/field/robot/camera").length);
  }

  @Test
  void autoPathOverlayFollowsTheSelection() {
    NtRecorder recorder = NtRecorder.getInstance();

    FieldScene.logAutoPath(recorder, List.of());
    assertEquals("rerun.archetypes.Clear", string("/Rerun/field/auto_path/.rerun/archetype"));

    FieldScene.logAutoPath(
        recorder,
        List.of(
            new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0.0)),
            new Pose2d(new Translation2d(3.0, 2.0), new Rotation2d(Math.PI / 2.0))));

    assertEquals(
        "rerun.archetypes.LineStrips2D", string("/Rerun/field/auto_path/.rerun/archetype"));
    assertArrayEquals(new long[] {2}, integers("/Rerun/field/auto_path/strips/.count"));
    assertArrayEquals(
        new double[] {1.0, 3.0}, doubles("/Rerun/field/auto_path/endpoints/positions/0"));
    assertTrue(topicsUnder("/Rerun/field/auto_path/start_heading").length > 0);
  }

  @Test
  void disabledVisualizerPublishesNothing() {
    System.setProperty("frc.robot.viz", "0");
    RerunVisualizer visualizer = new RerunVisualizer(new UnusedInputs());

    visualizer.periodic();
    visualizer.close();

    assertEquals(0, topicsUnder("/Rerun").length, "nothing should be published while disabled");
  }

  private static VizFrame frame(long loop) {
    double time = loop * 0.02;
    return new VizFrame(
        loop,
        time,
        new Pose2d(new Translation2d(time, 1.0 + Math.sin(time)), new Rotation2d(time)),
        new Pose2d(new Translation2d(time, 1.0), new Rotation2d(time)),
        new ChassisVelocities(1.0, 0.2, 0.1),
        0.01,
        loop % 2 == 0,
        "INTAKE",
        3200.0,
        "FEED",
        "SHOOT",
        2950.0,
        3000.0,
        31.5,
        true,
        new Translation2d(4.0, 4.0),
        new int[] {26, 27},
        true,
        "AIM_TARGET",
        "Safe",
        false,
        true,
        false,
        false,
        true,
        false);
  }

  private double[] doubles(String topic) {
    return instance.getEntry(topic).getDoubleArray(new double[0]);
  }

  private long[] integers(String topic) {
    return instance.getEntry(topic).getIntegerArray(new long[0]);
  }

  private String[] strings(String topic) {
    return instance.getEntry(topic).getStringArray(new String[0]);
  }

  private boolean bool(String topic) {
    return instance.getEntry(topic).getBoolean(false);
  }

  private long integer(String topic) {
    return instance.getEntry(topic).getInteger(0);
  }

  private String string(String topic) {
    return instance.getEntry(topic).getString("");
  }

  private org.wpilib.networktables.TopicInfo[] topicsUnder(String prefix) {
    return instance.getTopicInfo(prefix);
  }

  /** Frame-building tests never consult the robot; any access would fail loudly. */
  private static final class UnusedInputs implements VizInputs {
    @Override
    public Drive drive() {
      throw new AssertionError("unexpected robot access");
    }

    @Override
    public Intake intake() {
      throw new AssertionError("unexpected robot access");
    }

    @Override
    public Indexer indexer() {
      throw new AssertionError("unexpected robot access");
    }

    @Override
    public Shooter shooter() {
      throw new AssertionError("unexpected robot access");
    }

    @Override
    public Vision vision() {
      throw new AssertionError("unexpected robot access");
    }

    @Override
    public boolean redAlliance() {
      throw new AssertionError("unexpected robot access");
    }

    @Override
    public String driveMode() {
      throw new AssertionError("unexpected robot access");
    }

    @Override
    public String selectedAutoName() {
      throw new AssertionError("unexpected robot access");
    }

    @Override
    public List<Pose2d> selectedAutoPath() {
      throw new AssertionError("unexpected robot access");
    }

    @Override
    public boolean hubActive() {
      throw new AssertionError("unexpected robot access");
    }

    @Override
    public boolean hubActivatingSoon() {
      throw new AssertionError("unexpected robot access");
    }
  }
}
