// Copyright (c) 2021-2026 Littleton Robotics
// Use of this source code is governed by the BSD license in AdvantageKit.
package org.littletonrobotics.junction;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.RobotController;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.util.Color;
import org.wpilib.util.WPISerializable;
import org.wpilib.util.protobuf.Protobuf;
import org.wpilib.util.struct.Struct;
import org.wpilib.util.struct.StructSerializable;
import us.hebi.quickbuf.ProtoMessage;

/**
 * Host-thread AdvantageKit boundary for unchanged SIM. LogTable, generated inputs, automatic
 * outputs, and receivers are the pinned SDK implementations; native capture and receiver threads
 * are deliberately not used. Missing native-only measurements are absent, not synthetic zeros.
 */
public final class Logger {
  private static final LogTable entry = new LogTable(0);
  private static final Map<String, String> metadata = new LinkedHashMap<>();
  private static final List<LogDataReceiver> receivers = new ArrayList<>();
  private static final List<LoggedNetworkInput> dashboardInputs = new ArrayList<>();
  private static final float[][] joystickAxes = new float[6][];
  private static final int[][] joystickPovs = new int[6][];
  private static LogTable outputTable;
  private static ConsoleSource console;
  private static boolean enableConsole = true;
  private static boolean running;
  private static boolean ended;
  private static boolean receiverFault;
  private static int startedReceivers;
  private static long cycleCount;

  private Logger() {}

  public static void setReplaySource(LogReplaySource source) {
    if (!running && source != null) {
      throw new UnsupportedOperationException("The wasmbot AdvantageKit boundary supports SIM, not HAL replay");
    }
  }

  public static boolean hasReplaySource() {
    return false;
  }

  public static void addDataReceiver(LogDataReceiver receiver) {
    if (!running && !ended) receivers.add(Objects.requireNonNull(receiver));
  }

  public static void registerDashboardInput(LoggedNetworkInput input) {
    dashboardInputs.add(Objects.requireNonNull(input));
  }

  public static void registerURCL(Supplier<ByteBuffer[]> supplier) {
    throw new UnsupportedOperationException("Native REV/URCL capture is unavailable in wasmbot SIM");
  }

  public static void recordMetadata(String key, String value) {
    if (!running && !ended) metadata.put(key, value);
  }

  public static void disableConsoleCapture() {
    enableConsole = false;
  }

  public static void start() {
    if (running) return;
    if (ended) throw new IllegalStateException("AdvantageKit logging has already ended");
    outputTable = entry.getSubtable("RealOutputs");
    LogTable metadataTable = entry.getSubtable("RealMetadata");
    metadata.forEach(metadataTable::put);
    running = true;
    try {
      if (enableConsole && console == null) console = new ConsoleSource.Simulator();
      for (LogDataReceiver receiver : receivers) {
        // Include a receiver whose start throws in cleanup, since it may have acquired resources.
        startedReceivers++;
        receiver.start();
      }
      RobotController.setTimeSource(Logger::getTimestamp);
      periodicBeforeUser();
    } catch (RuntimeException | Error failure) {
      try {
        end();
      } catch (RuntimeException | Error cleanup) {
        failure.addSuppressed(cleanup);
      }
      throw failure;
    }
  }

  /** Stops every started receiver even when another receiver fails to close. */
  public static void end() {
    if (ended) return;
    running = false;
    ended = true;
    RobotController.setTimeSource(RobotController::getMonotonicTime);
    Throwable failure = null;
    if (console != null) {
      try {
        console.close();
      } catch (Exception | LinkageError exception) {
        failure = exception;
      }
      console = null;
    }
    for (int i = startedReceivers - 1; i >= 0; i--) {
      try {
        receivers.get(i).end();
      } catch (RuntimeException | Error exception) {
        if (failure == null) failure = exception;
        else failure.addSuppressed(exception);
      }
    }
    startedReceivers = 0;
    if (failure instanceof Error error) throw error;
    if (failure != null) throw new IllegalStateException("Failed to stop AdvantageKit logging", failure);
  }

  static void periodicBeforeUser() {
    cycleCount++;
    if (!running) return;
    long start = RobotController.getMonotonicTime();
    entry.setTimestamp(start);
    // LoggedNetworkNumber caches the host value here, before the user controls run.
    for (int i = 0; i < dashboardInputs.size(); i++) dashboardInputs.get(i).periodic();
    recordOutput("Logger/DashboardInputsMS", (RobotController.getMonotonicTime() - start) / 1_000_000.0);
  }

  static void periodicAfterUser(long userCodeLength, long periodicBeforeLength) {
    periodicAfterUser(userCodeLength, periodicBeforeLength, null);
  }

  static void periodicAfterUser(long userCodeLength, long periodicBeforeLength, String extraConsoleData) {
    if (!running) return;
    long start = RobotController.getMonotonicTime();
    saveDriverStation(entry.getSubtable("DriverStation"));
    long autoStart = RobotController.getMonotonicTime();
    AutoLogOutputManager.periodic();
    long autoEnd = RobotController.getMonotonicTime();
    String consoleData = enableConsole && console != null ? console.getNewData() : "";
    if (extraConsoleData != null) consoleData += extraConsoleData;
    if (!consoleData.isEmpty()) recordOutput("Console", consoleData.trim());
    long end = RobotController.getMonotonicTime();
    recordOutput("Logger/DriverStationMS", (autoStart - start) / 1_000_000.0);
    recordOutput("Logger/AutoLogMS", (autoEnd - autoStart) / 1_000_000.0);
    recordOutput("Logger/ConsoleMS", (end - autoEnd) / 1_000_000.0);
    recordOutput("LoggedRobot/UserCodeMS", userCodeLength / 1_000_000.0);
    recordOutput("LoggedRobot/LogPeriodicMS", (periodicBeforeLength + end - start) / 1_000_000.0);
    recordOutput("LoggedRobot/FullCycleMS", (periodicBeforeLength + userCodeLength + end - start) / 1_000_000.0);
    if (receivers.isEmpty()) return;
    // Receivers may retain a table (NT4Publisher uses the previous cycle for change detection).
    // One immutable cycle snapshot is shared just as in the SDK ReceiverThread.
    LogTable snapshot = LogTable.clone(entry);
    for (LogDataReceiver receiver : receivers) {
      try {
        receiver.putTable(snapshot);
      } catch (InterruptedException exception) {
        receiverFault = true;
        Thread.currentThread().interrupt();
        throw new IllegalStateException("AdvantageKit receiver interrupted", exception);
      } catch (RuntimeException | Error failure) {
        receiverFault = true;
        throw failure;
      }
    }
  }

  private static void saveDriverStation(LogTable table) {
    table.put("Enabled", RobotState.isEnabled());
    table.put("RobotMode", RobotState.getRobotMode());
    table.put("OpModeId", RobotState.getOpModeId());
    table.put("OpMode", RobotState.getOpMode());
    table.put("EmergencyStop", RobotState.isEStopped());
    table.put("FMSAttached", RobotState.isFMSAttached());
    table.put("DSAttached", RobotState.isDSAttached());
    // The host supplies alliance, not a station number, event identity, or match clock.
    table.put("Alliance", MatchState.getAlliance().map(Enum::name).orElse("Unknown"));
    for (int port = 0; port < 6; port++) {
      GenericHID hid = DriverStation.getGenericHID(port);
      LogTable joystick = table.getSubtable("Joystick" + port);
      joystick.put("Name", hid.getName());
      joystick.put("Connected", hid.isConnected());
      joystick.put("ButtonsAvailable", hid.getButtonsAvailable());
      long buttons = 0;
      for (int button = 0; button < hid.getButtonsMaximumIndex(); button++) {
        if (hid.getRawButton(button)) buttons |= 1L << button;
      }
      joystick.put("ButtonValues", buttons);
      joystick.put("AxesAvailable", hid.getAxesAvailable());
      int axisCount = hid.getAxesMaximumIndex();
      float[] axes = joystickAxes[port];
      if (axes == null || axes.length != axisCount) joystickAxes[port] = axes = new float[axisCount];
      for (int axis = 0; axis < axisCount; axis++) axes[axis] = (float) hid.getRawAxis(axis);
      joystick.put("AxisValues", axes);
      joystick.put("POVsAvailable", hid.getPOVsAvailable());
      int povCount = hid.getPOVsMaximumIndex();
      int[] povs = joystickPovs[port];
      if (povs == null || povs.length != povCount) joystickPovs[port] = povs = new int[povCount];
      for (int pov = 0; pov < povCount; pov++) povs[pov] = hid.getPOV(pov).value;
      joystick.put("POVValues", povs);
    }
  }

  public static boolean getReceiverQueueFault() {
    return receiverFault;
  }

  public static long getTimestamp() {
    return running ? entry.getTimestamp() : RobotController.getMonotonicTime();
  }

  public static void runEveryN(int n, Runnable function) {
    if (cycleCount % n == 0) function.run();
  }

  public static void processInputs(String key, LoggableInputs inputs) {
    if (running) inputs.toLog(entry.getSubtable(key));
  }

  /** Exact pinned AdvantageKit hooks, also used by the consumer's host smoke harness. */
  public static final class AdvancedHooks {
    private AdvancedHooks() {}
    public static void disableRobotBaseCheck() {
      // This boundary has no reflective stack check; LoggedRobot owns the host lifecycle.
    }
    public static void invokePeriodicBeforeUser() { periodicBeforeUser(); }
    public static void invokePeriodicAfterUser(long userCodeLength, long periodicBeforeLength) {
      periodicAfterUser(userCodeLength, periodicBeforeLength);
    }
    public static void invokePeriodicAfterUser(long userCodeLength, long periodicBeforeLength, String extraConsoleData) {
      periodicAfterUser(userCodeLength, periodicBeforeLength, extraConsoleData);
    }
    public static void setConsoleSource(ConsoleSource source) { console = source; }
  }

  // Keep the pinned SDK descriptors, not an Object catch-all: retained SDK classes are precompiled.
  public static void recordOutput(String key, byte[] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, byte[][] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, boolean value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, BooleanSupplier value) { if (running) outputTable.put(key, value.getAsBoolean()); }
  public static void recordOutput(String key, boolean[] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, boolean[][] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, int value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, IntSupplier value) { if (running) outputTable.put(key, value.getAsInt()); }
  public static void recordOutput(String key, int[] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, int[][] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, long value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, LongSupplier value) { if (running) outputTable.put(key, value.getAsLong()); }
  public static void recordOutput(String key, long[] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, long[][] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, float value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, float value, Unit unit) { if (running) outputTable.put(key, value, unit.name()); }
  public static void recordOutput(String key, float value, String unit) { if (running) outputTable.put(key, value, unit); }
  public static void recordOutput(String key, float[] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, float[][] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, double value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, double value, Unit unit) { if (running) outputTable.put(key, value, unit.name()); }
  public static void recordOutput(String key, double value, String unit) { if (running) outputTable.put(key, value, unit); }
  public static void recordOutput(String key, DoubleSupplier value) { if (running) outputTable.put(key, value.getAsDouble()); }
  public static void recordOutput(String key, double[] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, double[][] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, String value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, String[] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, String[][] value) { if (running) outputTable.put(key, value); }
  public static <E extends Enum<E>> void recordOutput(String key, E value) { if (running) outputTable.put(key, value); }
  public static <E extends Enum<E>> void recordOutput(String key, E[] value) { if (running) outputTable.put(key, value); }
  public static <E extends Enum<E>> void recordOutput(String key, E[][] value) { if (running) outputTable.put(key, value); }
  public static <U extends Unit> void recordOutputMeasure(String key, Measure<U> value) {
    if (running && value != null) outputTable.put(key, value.magnitude(), value.unit().name());
  }
  public static <T> void recordOutput(String key, Struct<T> struct, T value) { if (running) outputTable.put(key, struct, value); }
  @SafeVarargs
  public static <T> void recordOutput(String key, Struct<T> struct, T... value) { if (running) outputTable.put(key, struct, value); }
  public static <T> void recordOutput(String key, Struct<T> struct, T[][] value) { if (running) outputTable.put(key, struct, value); }
  public static <T, MessageType extends ProtoMessage<?>> void recordOutput(String key, Protobuf<T, MessageType> proto, T value) {
    if (running) outputTable.put(key, proto, value);
  }
  public static <T extends WPISerializable> void recordOutput(String key, T value) { if (running) outputTable.put(key, value); }
  @SafeVarargs
  public static <T extends StructSerializable> void recordOutput(String key, T... value) { if (running) outputTable.put(key, value); }
  public static <T extends StructSerializable> void recordOutput(String key, T[][] value) { if (running) outputTable.put(key, value); }
  @SuppressWarnings("unchecked")
  public static <R extends Record> void recordOutput(String key, R value) {
    if (value instanceof Measure<?> measure) recordOutputMeasure(key, (Measure<Unit>) measure);
    else if (running) outputTable.put(key, value);
  }
  @SafeVarargs
  public static <R extends Record> void recordOutput(String key, R... value) { if (running) outputTable.put(key, value); }
  public static <R extends Record> void recordOutput(String key, R[][] value) { if (running) outputTable.put(key, value); }
  public static void recordOutput(String key, LoggedMechanism2d value) { if (running) value.logOutput(outputTable.getSubtable(key)); }
  public static void recordOutput(String key, Color value) { if (running) outputTable.put(key, value); }
}
