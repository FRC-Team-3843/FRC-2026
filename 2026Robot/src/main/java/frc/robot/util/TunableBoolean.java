package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.TelemetryConstants;

/**
 * A bidirectional tunable boolean published to NetworkTables.
 *
 * <p>Same pattern as {@link TunableNumber} but for boolean enable/disable flags.
 * When {@link TelemetryConstants#TUNING_MODE} is false, returns the default value
 * with zero NT overhead.
 */
public class TunableBoolean {
  private final boolean m_defaultValue;
  private final NetworkTableEntry m_entry;
  private boolean m_lastValue;
  private final boolean m_tuningEnabled;

  /**
   * Create a tunable boolean.
   *
   * @param table subtable path under the specified root (e.g. "Turret/Left")
   * @param key the key name (e.g. "Enabled")
   * @param defaultValue the initial and fallback value
   * @param rootTable the root NT table name (e.g. "Control" or "Tuning")
   */
  public TunableBoolean(String rootTable, String table, String key, boolean defaultValue) {
    m_defaultValue = defaultValue;
    m_tuningEnabled = TelemetryConstants.TUNING_MODE;

    if (m_tuningEnabled) {
      m_entry = NetworkTableInstance.getDefault()
          .getTable(rootTable)
          .getSubTable(table)
          .getEntry(key);
      m_entry.setBoolean(defaultValue);
      m_lastValue = defaultValue;
    } else {
      m_entry = null;
      m_lastValue = defaultValue;
    }
  }

  /**
   * Get the current value. If tuning mode is enabled, reads from NT.
   * Otherwise returns the default value.
   */
  public boolean get() {
    if (m_tuningEnabled && m_entry != null) {
      return m_entry.getBoolean(m_defaultValue);
    }
    return m_defaultValue;
  }

  /**
   * Check if the value has changed since the last call to this method.
   */
  public boolean hasChanged() {
    if (!m_tuningEnabled || m_entry == null) {
      return false;
    }
    boolean current = m_entry.getBoolean(m_defaultValue);
    if (current != m_lastValue) {
      m_lastValue = current;
      return true;
    }
    return false;
  }

  /** Get the default value this TunableBoolean was created with. */
  public boolean getDefault() {
    return m_defaultValue;
  }
}
