package frc.robot.viz;

/**
 * WASM builds do not publish Rerun data.
 *
 * <p>The GraalVM Web Image cannot load the NetworkTables JNI natives moneyshift needs, so the wasm
 * source set excludes the moneyshift jars and this class shadows the desktop implementation on the
 * image classpath. Desktop and SystemCore builds use the real implementation. This is a
 * deliberate, documented exclusion — not a silent no-op for an integration expected to work.
 */
public final class RerunVisualizer {
  public RerunVisualizer(VizInputs inputs) {
    // Intentionally empty: nothing is published in WASM.
  }

  public void periodic() {}

  public void close() {}
}
