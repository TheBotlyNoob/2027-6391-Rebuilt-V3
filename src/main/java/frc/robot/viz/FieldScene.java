package frc.robot.viz;

import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import org.teamvan.moneyshift.NtRecorder;
import org.teamvan.moneyshift.archetypes.Arrows2D;
import org.teamvan.moneyshift.archetypes.Boxes2D;
import org.teamvan.moneyshift.archetypes.Clear;
import org.teamvan.moneyshift.archetypes.LineStrips2D;
import org.teamvan.moneyshift.archetypes.Points2D;
import org.teamvan.moneyshift.components.HalfSize2D;
import org.teamvan.moneyshift.components.LineStrip2D;
import org.teamvan.moneyshift.components.Position2D;
import org.teamvan.moneyshift.components.Vector2D;
import org.teamvan.moneyshift.encodings.Vec2D;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;

/**
 * Static 2D geometry of the 2026 REBUILT field plus the selected autonomous path.
 *
 * <p>All positions come from {@link FieldConstants} and the official field layout, so the drawing
 * matches the field the robot actually uses. Logged once over NetworkTables; the Rerun bridge
 * turns the topics into a chunk.
 */
final class FieldScene {
  private FieldScene() {}

  private static final double LENGTH = FieldConstants.fieldLength;
  private static final double WIDTH = FieldConstants.fieldWidth;

  static void logStatic(NtRecorder rec) {
    logField2d(rec);
    logTags(rec);
  }

  private static void logField2d(NtRecorder rec) {
    rec.logStatic(
        "field/boundary",
        LineStrips2D.builder()
            .withStrips(VizGeometry.rect2(0.0, 0.0, LENGTH, WIDTH))
            .withColors(VizGeometry.OUTLINE)
            .withRadii(0.03f)
            .build());

    // Alliance and neutral zone boundaries.
    rec.logStatic(
        "field/zones",
        LineStrips2D.builder()
            .withStrips(
                VizGeometry.strip2(
                    List.of(
                        VizGeometry.v2(FieldConstants.Zones.blueAllianceZoneX, 0.0),
                        VizGeometry.v2(FieldConstants.Zones.blueAllianceZoneX, WIDTH))),
                VizGeometry.strip2(
                    List.of(
                        VizGeometry.v2(FieldConstants.Zones.redAllianceZoneX, 0.0),
                        VizGeometry.v2(FieldConstants.Zones.redAllianceZoneX, WIDTH))),
                VizGeometry.rect2(
                    FieldConstants.Zones.neutralZoneStartX,
                    0.0,
                    FieldConstants.Zones.neutralZoneEndX,
                    WIDTH))
            .withColors(VizGeometry.BLUE, VizGeometry.RED, VizGeometry.OUTLINE)
            .withRadii(0.015f)
            .build());

    // Hubs (47" square, centred on the hub coordinate).
    double hubHalf = FieldConstants.Hub.width / 2.0;
    rec.logStatic(
        "field/hubs",
        Boxes2D.builder()
            .withCenters(
                Position2D.of(
                    (float) FieldConstants.Hub.blueCenter.getX(),
                    (float) FieldConstants.Hub.blueCenter.getY()),
                Position2D.of(
                    (float) FieldConstants.Hub.redCenter.getX(),
                    (float) FieldConstants.Hub.redCenter.getY()))
            .withHalfSizes(HalfSize2D.of((float) hubHalf, (float) hubHalf))
            .withColors(VizGeometry.HUB_BLUE, VizGeometry.HUB_RED)
            .withLabels("BLUE HUB", "RED HUB")
            .build());

    // Trench and depot structures.
    rec.logStatic(
        "field/trenches",
        LineStrips2D.builder()
            .withStrips(
                trenchRect(FieldConstants.Trench.blueTopCenter),
                trenchRect(FieldConstants.Trench.blueBottomCenter),
                trenchRect(FieldConstants.Trench.redTopCenter),
                trenchRect(FieldConstants.Trench.redBottomCenter))
            .withColors(VizGeometry.TRENCH)
            .withRadii(0.02f)
            .build());

    rec.logStatic(
        "field/depots",
        LineStrips2D.builder()
            .withStrips(
                depotRect(FieldConstants.Depot.blueCenter),
                depotRect(FieldConstants.Depot.redCenter))
            .withColors(VizGeometry.DEPOT)
            .withRadii(0.02f)
            .build());
  }

  private static LineStrip2D trenchRect(org.wpilib.math.geometry.Translation2d center) {
    double halfWidth = FieldConstants.Trench.width / 2.0;
    double halfDepth = FieldConstants.Trench.depth / 2.0;
    return VizGeometry.rect2(
        center.getX() - halfWidth,
        center.getY() - halfDepth,
        center.getX() + halfWidth,
        center.getY() + halfDepth);
  }

  private static LineStrip2D depotRect(org.wpilib.math.geometry.Translation2d center) {
    // Depot is 42" along the alliance wall (Y) and 27" deep (X).
    double halfDepth = FieldConstants.Depot.depth / 2.0;
    double halfWidth = FieldConstants.Depot.width / 2.0;
    return VizGeometry.rect2(
        center.getX() - halfDepth,
        center.getY() - halfWidth,
        center.getX() + halfDepth,
        center.getY() + halfWidth);
  }

  /** AprilTag positions from the loaded field layout, projected onto the field plane. */
  private static void logTags(NtRecorder rec) {
    List<Position2D> points = new ArrayList<>();
    List<String> labels = new ArrayList<>();

    for (FieldTag tag : VisionConstants.aprilTagLayout.getTags()) {
      Pose3d pose = tag.getPose();
      points.add(Position2D.of((float) pose.getX(), (float) pose.getY()));
      labels.add(Integer.toString(tag.getID()));
    }
    if (points.isEmpty()) {
      return;
    }

    rec.logStatic(
        "field/tags",
        Points2D.builder()
            .withPositions(points)
            .withLabels(labels.toArray(new String[0]))
            .withColors(VizGeometry.TAG)
            .withRadii(0.06f)
            .build());
  }

  /**
   * Draws the selected autonomous routine. Re-logged only when the selection or alliance changes.
   * An empty list clears the previous path.
   */
  static void logAutoPath(NtRecorder rec, List<Pose2d> poses) {
    if (poses == null || poses.isEmpty()) {
      // No routine selected: drop the overlay (and its child entities).
      rec.log("field/auto_path", Clear.builder().withIsRecursive(true).build());
      return;
    }
    List<Vec2D> points = new ArrayList<>(poses.size());
    for (Pose2d pose : poses) {
      points.add(VizGeometry.v2(pose.getX(), pose.getY()));
    }
    rec.log(
        "field/auto_path",
        LineStrips2D.builder()
            .withStrips(VizGeometry.strip2(points))
            .withColors(VizGeometry.PATH)
            .withRadii(0.03f)
            .build());

    // Start and end markers, plus a heading arrow at the start pose.
    Pose2d start = poses.get(0);
    Pose2d end = poses.get(poses.size() - 1);
    rec.log(
        "field/auto_path/endpoints",
        Points2D.builder()
            .withPositions(
                Position2D.of((float) start.getX(), (float) start.getY()),
                Position2D.of((float) end.getX(), (float) end.getY()))
            .withColors(VizGeometry.PATH, VizGeometry.TARGET)
            .withRadii(0.09f)
            .withLabels("START", "END")
            .build());
    rec.log(
        "field/auto_path/start_heading",
        Arrows2D.builder()
            .withOrigins(Position2D.of((float) start.getX(), (float) start.getY()))
            .withVectors(
                Vector2D.of(
                    (float) (0.5 * start.getRotation().getCos()),
                    (float) (0.5 * start.getRotation().getSin())))
            .withColors(VizGeometry.PATH)
            .withRadii(0.02f)
            .build());
  }
}
