package frc.robot.viz;

import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import org.teamvan.moneyshift.NtRecorder;
import org.teamvan.moneyshift.archetypes.Arrows2D;
import org.teamvan.moneyshift.archetypes.LineStrips2D;
import org.teamvan.moneyshift.archetypes.Points2D;
import org.teamvan.moneyshift.components.Position2D;
import org.teamvan.moneyshift.components.Vector2D;
import org.teamvan.moneyshift.encodings.Vec2D;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * Dynamic robot geometry for the top-down view: the swerve footprint, heading, recent trail, the
 * current aim target, and the AprilTags the camera can see.
 *
 * <p>There is no 3D robot model: the visualization is field-plane only. The drive chassis size
 * comes from the configured swerve module locations.
 */
final class RobotScene {
  /** Roughly twenty seconds of logged history at the default visualization rate. */
  private static final int TRAIL_LENGTH = 200;

  private final Deque<Vec2D> trail = new ArrayDeque<>(TRAIL_LENGTH + 1);

  void log(NtRecorder rec, VizFrame frame) {
    logFootprint(rec, frame);
    logObservedTags(rec, frame);
  }

  private void logFootprint(NtRecorder rec, VizFrame frame) {
    Pose2d pose = frame.pose();
    double hx = VizGeometry.CHASSIS_HALF_X;
    double hy = VizGeometry.CHASSIS_HALF_Y;

    List<Vec2D> corners = new ArrayList<>(5);
    corners.add(corner(pose, hx, hy));
    corners.add(corner(pose, hx, -hy));
    corners.add(corner(pose, -hx, -hy));
    corners.add(corner(pose, -hx, hy));
    corners.add(corner(pose, hx, hy));

    rec.log(
        "field/robot",
        LineStrips2D.builder()
            .withStrips(VizGeometry.strip2(corners))
            .withColors(VizGeometry.ROBOT)
            .withRadii(0.03f)
            .build());

    double heading = pose.getRotation().getRadians();
    rec.log(
        "field/robot/heading",
        Arrows2D.builder()
            .withOrigins(Position2D.of((float) pose.getX(), (float) pose.getY()))
            .withVectors(
                Vector2D.of(
                    (float) (0.55 * Math.cos(heading)), (float) (0.55 * Math.sin(heading))))
            .withColors(VizGeometry.HEADING)
            .withRadii(0.025f)
            .build());

    if (trail.size() >= TRAIL_LENGTH) {
      trail.removeFirst();
    }
    trail.addLast(VizGeometry.v2(pose.getX(), pose.getY()));
    // The trail is the most expensive entity (hundreds of points per update), so log it rarely.
    if (frame.loop() % 50 == 0 && trail.size() > 1) {
      rec.log(
          "field/robot/trail",
          LineStrips2D.builder()
              .withStrips(VizGeometry.strip2(new ArrayList<>(trail)))
              .withColors(VizGeometry.TRAIL)
              .withRadii(0.012f)
              .build());
    }

    rec.log(
        "field/aim_target",
        Points2D.builder()
            .withPositions(
                Position2D.of(
                    (float) frame.aimTarget().getX(), (float) frame.aimTarget().getY()))
            .withColors(VizGeometry.TARGET)
            .withRadii(0.07f)
            .withLabels("AIM")
            .build());
  }

  private static Vec2D corner(Pose2d pose, double dx, double dy) {
    var translation =
        pose.transformBy(new Transform2d(dx, dy, Rotation2d.ZERO)).getTranslation();
    return VizGeometry.v2(translation.getX(), translation.getY());
  }

  /** Tags the camera currently reports, drawn at their field-plane position. */
  private void logObservedTags(NtRecorder rec, VizFrame frame) {
    int[] ids = frame.tagIds();
    List<Position2D> positions = new ArrayList<>(ids.length);
    List<String> labels = new ArrayList<>(ids.length);
    for (int id : ids) {
      var tagPose = VisionConstants.aprilTagLayout.getTagPose(id);
      if (tagPose.isEmpty()) {
        continue;
      }
      Pose3d pose = tagPose.get();
      positions.add(Position2D.of((float) pose.getX(), (float) pose.getY()));
      labels.add(Integer.toString(id));
    }
    if (positions.isEmpty()) {
      return;
    }
    rec.log(
        "field/tags/observed",
        Points2D.builder()
            .withPositions(positions)
            .withLabels(labels.toArray(new String[0]))
            .withColors(VizGeometry.GOOD)
            .withRadii(0.09f)
            .build());
  }
}
