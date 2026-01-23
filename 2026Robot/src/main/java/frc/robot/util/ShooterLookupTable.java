package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/**
 * Loads shooter RPM/hood angle settings from a JSON lookup table.
 *
 * <p>This is a stub for future shooter work. It is safe to keep unused until
 * shooter hardware is ready.
 */
public final class ShooterLookupTable {

  public static final class Units {
    public String distance;
    public String rpm;
    public String hoodAngle;
  }

  public static final class Point {
    public double distance;
    public double rpm;
    public double hoodAngle;
  }

  public static final class Table {
    public int version;
    public Units units;
    public List<Point> points;
  }

  private final Table table;

  private ShooterLookupTable(Table table) {
    this.table = table;
  }

  public static ShooterLookupTable load(Path jsonPath) {
    if (!Files.exists(jsonPath)) {
      DriverStation.reportWarning("Shooter table not found: " + jsonPath, false);
      return new ShooterLookupTable(null);
    }

    ObjectMapper mapper = new ObjectMapper();
    try {
      Table table = mapper.readValue(jsonPath.toFile(), Table.class);
      return new ShooterLookupTable(table);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load shooter table: " + e.getMessage(), e.getStackTrace());
      return new ShooterLookupTable(null);
    }
  }

  public boolean isValid() {
    return table != null && table.points != null && !table.points.isEmpty();
  }

  public Table getTable() {
    return table;
  }
}
