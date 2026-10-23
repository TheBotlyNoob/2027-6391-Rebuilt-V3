package frc.robot.viz;

import java.util.ArrayList;
import java.util.List;
import org.teamvan.moneyshift.components.LineStrip2D;
import org.teamvan.moneyshift.encodings.Vec2D;

/**
 * Colours, robot dimensions, and small 2D geometry helpers shared by the Rerun scene.
 *
 * <p>Field dimensions come from {@link frc.robot.FieldConstants} (the 2026 REBUILT layout). Robot
 * dimensions are derived from {@link frc.robot.generated.TunerConstants}.
 */
final class VizGeometry {
  private VizGeometry() {}

  // Packed RGBA (0xRRGGBBAA), matching the Rerun color component layout.
  static final long OUTLINE = 0x9CA3AFFF;
  static final long BLUE = 0x3B82F6FF;
  static final long RED = 0xEF4444FF;
  static final long HUB_BLUE = 0x1D4ED8FF;
  static final long HUB_RED = 0xB91C1CFF;
  static final long TRENCH = 0x6B7280FF;
  static final long DEPOT = 0xA855F7FF;
  static final long ROBOT = 0x22D3EEFF;
  static final long HEADING = 0x00E5FFFF;
  static final long PATH = 0xF59E0BFF;
  static final long TARGET = 0x32D74BFF;
  static final long TRAIL = 0x22D3EE66;
  static final long TAG = 0xFBBF24FF;
  static final long INTAKE = 0x84CC16FF;
  static final long INDEXER = 0xF97316FF;
  static final long SHOOTER = 0xEF4444FF;
  static final long WARN = 0xF59E0BFF;
  static final long GOOD = 0x32D74BFF;

  /** Half-extents of the swerve chassis, from the module locations in TunerConstants. */
  static final double CHASSIS_HALF_X =
      Math.max(
              Math.max(
                  Math.abs(frc.robot.generated.TunerConstants.FrontLeft.LocationX),
                  Math.abs(frc.robot.generated.TunerConstants.BackLeft.LocationX)),
              Math.max(
                  Math.abs(frc.robot.generated.TunerConstants.FrontRight.LocationX),
                  Math.abs(frc.robot.generated.TunerConstants.BackRight.LocationX)))
          + 0.09;

  /** Half-extents of the swerve chassis, from the module locations in TunerConstants. */
  static final double CHASSIS_HALF_Y =
      Math.max(
              Math.max(
                  Math.abs(frc.robot.generated.TunerConstants.FrontLeft.LocationY),
                  Math.abs(frc.robot.generated.TunerConstants.FrontRight.LocationY)),
              Math.max(
                  Math.abs(frc.robot.generated.TunerConstants.BackLeft.LocationY),
                  Math.abs(frc.robot.generated.TunerConstants.BackRight.LocationY)))
          + 0.09;

  static Vec2D v2(double x, double y) {
    return Vec2D.of((float) x, (float) y);
  }

  /** A closed rectangle outline in the field plane. */
  static LineStrip2D rect2(double x0, double y0, double x1, double y1) {
    List<Vec2D> points = new ArrayList<>(5);
    points.add(v2(x0, y0));
    points.add(v2(x1, y0));
    points.add(v2(x1, y1));
    points.add(v2(x0, y1));
    points.add(v2(x0, y0));
    return new LineStrip2D(points);
  }

  static LineStrip2D strip2(List<Vec2D> points) {
    return new LineStrip2D(points);
  }
}
