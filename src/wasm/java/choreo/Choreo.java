// Copyright (c) Choreo contributors
// WASM adaptation: read bundled deploy resources; retain the pinned trajectory data model.
package choreo;

import choreo.trajectory.DifferentialSample;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import choreo.util.TrajSchemaVersion;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

/** Choreo loading from the WASM image's packaged src/main/deploy assets, without a host filesystem. */
public final class Choreo {
  private Choreo() {}

  @SuppressWarnings("unchecked")
  public static <S extends TrajectorySample<S>> Optional<Trajectory<S>> loadTrajectory(String name) {
    Objects.requireNonNull(name, "trajectoryName");
    String file = name.endsWith(".traj") ? name : name + ".traj";
    String resource = "/wasmbot/deploy/choreo/" + file;
    try (var stream = Choreo.class.getResourceAsStream(resource)) {
      if (stream == null) {
        System.err.println("Could not find Choreo trajectory: " + resource);
        return Optional.empty();
      }
      try (var reader = new InputStreamReader(stream, StandardCharsets.UTF_8)) {
        return Optional.of((Trajectory<S>) parse(JsonParser.parseReader(reader).getAsJsonObject()));
      }
    } catch (IOException | RuntimeException error) {
      System.err.println("Could not parse Choreo trajectory " + resource + ": " + error);
      return Optional.empty();
    }
  }

  static Trajectory<? extends TrajectorySample<?>> loadTrajectoryString(String json) {
    return parse(JsonParser.parseString(json).getAsJsonObject());
  }

  private static Trajectory<? extends TrajectorySample<?>> parse(JsonObject root) {
    String name = root.get("name").getAsString();
    int version = root.get("version").getAsInt();
    if (version != TrajSchemaVersion.TRAJ_SCHEMA_VERSION) {
      throw new IllegalArgumentException(
          name + ".traj: Wrong version: " + version + ". Expected "
              + TrajSchemaVersion.TRAJ_SCHEMA_VERSION);
    }
    var events = new ArrayList<EventMarker>();
    var deserializer = new EventMarker.Deserializer();
    for (JsonElement element : root.getAsJsonArray("events")) {
      EventMarker event = deserializer.deserialize(element, EventMarker.class, null);
      if (event.timestamp >= 0 && !event.event.isEmpty()) events.add(event);
    }
    JsonObject trajectory = root.getAsJsonObject("trajectory");
    var splits = new ArrayList<Integer>();
    for (JsonElement element : trajectory.getAsJsonArray("splits")) splits.add(element.getAsInt());
    if (splits.isEmpty() || splits.getFirst() != 0) splits.addFirst(0);
    JsonArray samples = trajectory.getAsJsonArray("samples");
    return switch (trajectory.get("sampleType").getAsString()) {
      case "Swerve" -> {
        var parsed = new ArrayList<SwerveSample>(samples.size());
        for (JsonElement element : samples) {
          JsonObject s = element.getAsJsonObject();
          parsed.add(new SwerveSample(
              number(s, "t"), number(s, "x"), number(s, "y"), number(s, "heading"),
              number(s, "vx"), number(s, "vy"), number(s, "omega"), number(s, "ax"),
              number(s, "ay"), number(s, "alpha"), array(s, "fx"), array(s, "fy")));
        }
        yield new Trajectory<>(name, List.copyOf(parsed), List.copyOf(splits), List.copyOf(events));
      }
      case "Differential" -> {
        var parsed = new ArrayList<DifferentialSample>(samples.size());
        for (JsonElement element : samples) {
          JsonObject s = element.getAsJsonObject();
          parsed.add(new DifferentialSample(
              number(s, "t"), number(s, "x"), number(s, "y"), number(s, "heading"),
              number(s, "vl"), number(s, "vr"), number(s, "omega"), number(s, "al"),
              number(s, "ar"), number(s, "alpha"), number(s, "fl"), number(s, "fr")));
        }
        yield new Trajectory<>(name, List.copyOf(parsed), List.copyOf(splits), List.copyOf(events));
      }
      default -> throw new IllegalArgumentException("Unknown drive type: " + trajectory.get("sampleType"));
    };
  }

  // Match Gson's default primitive values and the library's nullable force arrays without reflection.
  private static double number(JsonObject sample, String key) {
    JsonElement value = sample.get(key);
    return value == null || value.isJsonNull() ? 0.0 : value.getAsDouble();
  }

  private static double[] array(JsonObject sample, String key) {
    JsonElement value = sample.get(key);
    if (value == null || value.isJsonNull()) return null;
    JsonArray array = value.getAsJsonArray();
    double[] result = new double[array.size()];
    for (int i = 0; i < result.length; i++) result[i] = array.get(i).getAsDouble();
    return result;
  }
}
