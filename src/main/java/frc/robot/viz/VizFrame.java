package frc.robot.viz;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * One visualization frame: everything the Rerun scene needs, captured once per robot loop.
 *
 * <p>Values are flattened (no subsystem references) so the scene code can be exercised without
 * constructing hardware IO.
 */
record VizFrame(
    long loop,
    double robotTimeSecs,
    Pose2d pose,
    Pose2d simulationPose,
    ChassisVelocities fieldSpeeds,
    double pitchRad,
    boolean intakeDeployed,
    String intakeGoal,
    double intakeRollerRpm,
    String indexerGoal,
    String shooterGoal,
    double shooterRpm,
    double shooterSetpointRpm,
    double hoodDeg,
    boolean shooterAtSetpoint,
    Translation2d aimTarget,
    int[] tagIds,
    boolean tagsInView,
    String driveMode,
    String autoName,
    boolean redAlliance,
    boolean hubActive,
    boolean hubActivatingSoon,
    boolean autonomous,
    boolean teleop,
    boolean disabled) {

  VizFrame {
    tagIds = tagIds.clone();
  }
}
