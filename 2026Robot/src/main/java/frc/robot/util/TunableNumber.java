package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.TelemetryConstants;

/**
 * A bidirectional tunable number published to NetworkTables.
 *
 * <p>When {@link TelemetryConstants#TUNING_MODE} is true, the value is published to NT under
 * the {@code /Tuning/} table and can be edited live from Elastic Dashboard. When tuning mode
 * is false, {@link #get()} returns the default value with zero NT overhead.
 *
 * <p>Usage:
 * <pre>
 *   TunableNumber driveP = new TunableNumber("Swerve/Drive", "P", 0.1);
 *   // In periodic: double p = driveP.get();
 *   // Check if dashboard user changed it: if (driveP.hasChanged()) { reapply(); }
 * </pre>
 */
public class TunableNumber {
  private final double m_defaultValue;
  private final NetworkTableEntry m_entry;
  private double m_lastValue;
  private final boolean m_tuningEnabled;

  /**
   * Create a tunable number.
   *
   * @param table subtable path under /Tuning/ (e.g. "Swerve/Drive")
   * @param key the key name (e.g. "P")
   * @param defaultValue the initial and fallback value
   */
  public TunableNumber(String table, String key, double defaultValue) {
    m_defaultValue = defaultValue;
    m_tuningEnabled = TelemetryConstants.TUNING_MODE;

    if (m_tuningEnabled) {
      m_entry = NetworkTableInstance.getDefault()
          .getTable("Tuning")
          .getSubTable(table)
          .getEntry(key);
      m_entry.setDouble(defaultValue);
      m_lastValue = defaultValue;
    } else {
      m_entry = null;
      m_lastValue = defaultValue;
    }
  }

  /**
   * Get the current value. If tuning mode is enabled, reads from NT (reflecting any
   * dashboard edits). Otherwise returns the default value.
   */
  public double get() {
    if (m_tuningEnabled && m_entry != null) {
      return m_entry.getDouble(m_defaultValue);
    }
    return m_defaultValue;
  }

  /**
   * Check if the value has changed since the last call to this method.
   * Useful for detecting when a dashboard user edits the value so you can reapply PID gains.
   */
  public boolean hasChanged() {
    if (!m_tuningEnabled || m_entry == null) {
      return false;
    }
    double current = m_entry.getDouble(m_defaultValue);
    if (current != m_lastValue) {
      m_lastValue = current;
      return true;
    }
    return false;
  }

  /** Get the default value this TunableNumber was created with. */
  public double getDefault() {
    return m_defaultValue;
  }
}
