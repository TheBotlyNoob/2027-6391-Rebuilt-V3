package frc.robot.viz;

import java.util.Objects;
import org.teamvan.moneyshift.NtRecorder;
import org.teamvan.moneyshift.archetypes.Scalars;
import org.teamvan.moneyshift.archetypes.TextDocument;
import org.teamvan.moneyshift.archetypes.TextLog;

/**
 * Numeric plots, a state summary panel, and a discrete event log.
 *
 * <p>Plots mirror the values AdvantageKit already records for the same signals, so the Rerun
 * view is self-contained rather than a reference into a separate log. Each series is its own
 * entity: the NetworkTables protocol carries one archetype per update, so a multi-series
 * {@code SeriesLines} entity cannot also carry its {@code Scalars}.
 */
final class DataScene {
  private String lastIntakeGoal;
  private String lastIndexerGoal;
  private String lastShooterGoal;
  private boolean lastAtSetpoint;
  private boolean lastTagsInView;
  private boolean lastHubActive;
  private String lastMatchState;

  void log(NtRecorder rec, VizFrame frame) {
    logPlots(rec, frame);
    logEvents(rec, frame);
    // Roughly once per second at the default visualization rate.
    if (frame.loop() % 50 == 0) {
      logStatePanel(rec, frame);
    }
  }

  private static void logPlots(NtRecorder rec, VizFrame frame) {
    rec.log(
        "plots/ShooterRPM/measured",
        Scalars.builder().withScalars(frame.shooterRpm()).build());
    rec.log(
        "plots/ShooterRPM/setpoint",
        Scalars.builder().withScalars(frame.shooterSetpointRpm()).build());
    rec.log("plots/ShooterHoodDeg", Scalars.builder().withScalars(frame.hoodDeg()).build());

    double speed = Math.hypot(frame.fieldSpeeds().vx, frame.fieldSpeeds().vy);
    rec.log("plots/DriveSpeed", Scalars.builder().withScalars(speed).build());
    rec.log(
        "plots/DriveOmegaDegPerSec",
        Scalars.builder().withScalars(Math.toDegrees(frame.fieldSpeeds().omega)).build());
    rec.log(
        "plots/PitchDeg",
        Scalars.builder().withScalars(Math.toDegrees(frame.pitchRad())).build());
    rec.log(
        "plots/IntakeRollerRPM",
        Scalars.builder().withScalars(frame.intakeRollerRpm()).build());
  }

  private void logEvents(NtRecorder rec, VizFrame frame) {
    if (!Objects.equals(lastIntakeGoal, frame.intakeGoal())) {
      event(rec, "intake goal -> " + frame.intakeGoal(), VizGeometry.INTAKE);
      lastIntakeGoal = frame.intakeGoal();
    }
    if (!Objects.equals(lastIndexerGoal, frame.indexerGoal())) {
      event(rec, "indexer goal -> " + frame.indexerGoal(), VizGeometry.INDEXER);
      lastIndexerGoal = frame.indexerGoal();
    }
    if (!Objects.equals(lastShooterGoal, frame.shooterGoal())) {
      event(rec, "shooter goal -> " + frame.shooterGoal(), VizGeometry.SHOOTER);
      lastShooterGoal = frame.shooterGoal();
    }
    if (lastAtSetpoint != frame.shooterAtSetpoint()) {
      event(
          rec,
          frame.shooterAtSetpoint() ? "shooter at setpoint" : "shooter left setpoint",
          frame.shooterAtSetpoint() ? VizGeometry.GOOD : VizGeometry.WARN);
      lastAtSetpoint = frame.shooterAtSetpoint();
    }
    if (lastTagsInView != frame.tagsInView()) {
      event(
          rec,
          frame.tagsInView() ? "april tags in view" : "april tags lost",
          frame.tagsInView() ? VizGeometry.GOOD : VizGeometry.WARN);
      lastTagsInView = frame.tagsInView();
    }
    if (lastHubActive != frame.hubActive()) {
      event(
          rec,
          frame.hubActive() ? "hub active" : "hub inactive",
          frame.hubActive() ? VizGeometry.GOOD : VizGeometry.WARN);
      lastHubActive = frame.hubActive();
    }

    String matchState = matchState(frame);
    if (!Objects.equals(lastMatchState, matchState)) {
      event(rec, "mode -> " + matchState, VizGeometry.OUTLINE);
      lastMatchState = matchState;
    }
  }

  private static String matchState(VizFrame frame) {
    if (frame.disabled()) {
      return "DISABLED";
    }
    if (frame.autonomous()) {
      return "AUTONOMOUS";
    }
    if (frame.teleop()) {
      return "TELEOP";
    }
    return "UNKNOWN";
  }

  private static void event(NtRecorder rec, String text, long color) {
    rec.log(
        "logs/events",
        TextLog.builder().withText(text).withLevel("INFO").withColor(color).build());
  }

  private void logStatePanel(NtRecorder rec, VizFrame frame) {
    String markdown =
        String.format(
            """
            | | |
            |---|---|
            | Mode | %s |
            | Alliance | %s |
            | Drive mode | %s |
            | Auto | %s |
            | Pose | %.2f, %.2f, %.1f deg |
            | Speed | %.2f m/s |
            | Hub | %s%s |
            | Intake | %s%s |
            | Indexer | %s |
            | Shooter | %s, %.0f / %.0f RPM, hood %.1f deg |
            | Vision | %s |
            """,
            matchState(frame),
            frame.redAlliance() ? "RED" : "BLUE",
            frame.driveMode(),
            frame.autoName(),
            frame.pose().getX(),
            frame.pose().getY(),
            frame.pose().getRotation().getDegrees(),
            Math.hypot(frame.fieldSpeeds().vx, frame.fieldSpeeds().vy),
            frame.hubActive() ? "active" : "inactive",
            frame.hubActivatingSoon() ? " (activating soon)" : "",
            frame.intakeGoal(),
            frame.intakeDeployed() ? ", deployed" : ", stowed",
            frame.indexerGoal(),
            frame.shooterGoal(),
            frame.shooterRpm(),
            frame.shooterSetpointRpm(),
            frame.hoodDeg(),
            frame.tagsInView() ? "tags in view" : "no tags");

    rec.log(
        "logs/state",
        TextDocument.builder().withText(markdown).withMediaType("text/markdown").build());
  }
}
