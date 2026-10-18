// Copyright (c) 2021-2026 Littleton Robotics
// Use of this source code is governed by the BSD license in AdvantageKit.
package org.littletonrobotics.junction;

import java.io.PrintWriter;
import java.io.StringWriter;
import org.wpilib.framework.IterativeRobotBase;
import org.wpilib.system.RobotController;

/**
 * AdvantageKit's iterative lifecycle driven by serialized wasmbot host ticks, without native
 * notifiers, Conduit, management beans, or a blocking competition loop.
 */
public class LoggedRobot extends IterativeRobotBase {
  public static final double defaultPeriodSecs = 0.02;

  private final long periodNs;
  private long nextCycleNs;
  private boolean useTiming = true;

  protected LoggedRobot() {
    this(defaultPeriodSecs);
  }

  protected LoggedRobot(double period) {
    super(period);
    periodNs = (long) (period * 1_000_000_000.0);
  }

  /** Runs initialization once and returns so that the host can deliver the next tick. */
  @Override
  public void startCompetition() {
    if (!beginCompetition()) return;
    try {
      long initStart = RobotController.getMonotonicTime();
      simulationInit();
      long initEnd = RobotController.getMonotonicTime();
      // Construction has completed, including RobotContainer and all subsystem fields.
      AutoLogOutputManager.addObject(this);
      Logger.periodicAfterUser(initEnd - initStart, 0);
      System.out.println("********** Robot program startup complete **********");
      nextCycleNs = RobotController.getMonotonicTime() + periodNs;
    } catch (RuntimeException | Error failure) {
      failCompetition(failure);
      throw failure;
    }
  }

  /** Before-user input capture, standard mode callbacks, and after-user output capture. */
  @Override
  protected void wasmLoop() {
    long now = RobotController.getMonotonicTime();
    if (useTiming && now < nextCycleNs) return;
    // Skip missed deadlines rather than burst-replaying control code after a host pause.
    if (useTiming) {
      nextCycleNs += periodNs + (now - nextCycleNs) / periodNs * periodNs;
    } else {
      nextCycleNs = now + periodNs;
    }
    try {
      long beforeStart = RobotController.getMonotonicTime();
      Logger.periodicBeforeUser();
      long userStart = RobotController.getMonotonicTime();
      loopFunc();
      long userEnd = RobotController.getMonotonicTime();
      Logger.periodicAfterUser(userEnd - userStart, userStart - beforeStart);
    } catch (RuntimeException | Error failure) {
      failCompetition(failure);
      throw failure;
    }
  }

  private void failCompetition(Throwable failure) {
    stopCompetition();
    try {
      StringWriter text = new StringWriter();
      failure.printStackTrace(new PrintWriter(text));
      Logger.periodicAfterUser(0, 0, text.toString());
    } catch (RuntimeException | Error loggingFailure) {
      failure.addSuppressed(loggingFailure);
    }
    try {
      Logger.end();
    } catch (RuntimeException | Error cleanupFailure) {
      failure.addSuppressed(cleanupFailure);
    }
  }

  @Override
  public void endCompetition() {
    stopCompetition();
    Logger.end();
  }

  @Override
  public void close() {
    endCompetition();
  }

  /** Disabling timing runs at most once per host tick; it never creates a busy loop. */
  public void setUseTiming(boolean useTiming) {
    this.useTiming = useTiming;
  }
}
