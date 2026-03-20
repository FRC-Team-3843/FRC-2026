package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Manages hood servo positions via a JSON file on the RoboRIO.
 *
 * <p>Four positions: left near, left far, right near, right far.
 * Values are loaded from deploy/hood-config.json on construction,
 * published to SmartDashboard for live tuning, and saved back to
 * the JSON file when the dashboard save button is pressed.
 */
public class HoodConfig {

  private static final String CONFIG_FILE = "hood-config.json";

  private double m_leftNearAngle;
  private double m_leftFarAngle;
  private double m_rightNearAngle;
  private double m_rightFarAngle;

  private final File m_configFile;

  public HoodConfig() {
    m_configFile = new File(Filesystem.getDeployDirectory(), CONFIG_FILE);
    load();

    // Publish to dashboard for tuning
    SmartDashboard.putNumber("Hood/LeftNearAngle", m_leftNearAngle);
    SmartDashboard.putNumber("Hood/LeftFarAngle", m_leftFarAngle);
    SmartDashboard.putNumber("Hood/RightNearAngle", m_rightNearAngle);
    SmartDashboard.putNumber("Hood/RightFarAngle", m_rightFarAngle);
    SmartDashboard.putBoolean("Hood/Save", false);
  }

  /** Read current values from dashboard and check save button. Call from periodic. */
  public void update() {
    m_leftNearAngle  = SmartDashboard.getNumber("Hood/LeftNearAngle", m_leftNearAngle);
    m_leftFarAngle   = SmartDashboard.getNumber("Hood/LeftFarAngle", m_leftFarAngle);
    m_rightNearAngle = SmartDashboard.getNumber("Hood/RightNearAngle", m_rightNearAngle);
    m_rightFarAngle  = SmartDashboard.getNumber("Hood/RightFarAngle", m_rightFarAngle);

    // Save button — write to JSON when pressed, then reset
    if (SmartDashboard.getBoolean("Hood/Save", false)) {
      save();
      SmartDashboard.putBoolean("Hood/Save", false);
    }
  }

  // ── Getters ──────────────────────────────────────────────────────────────

  public double getLeftAngle(boolean far) {
    return far ? m_leftFarAngle : m_leftNearAngle;
  }

  public double getRightAngle(boolean far) {
    return far ? m_rightFarAngle : m_rightNearAngle;
  }

  // ── JSON I/O ─────────────────────────────────────────────────────────────

  @SuppressWarnings("unchecked")
  private void load() {
    // Defaults
    m_leftNearAngle  = 45.0;
    m_leftFarAngle   = 90.0;
    m_rightNearAngle = 45.0;
    m_rightFarAngle  = 90.0;

    if (!m_configFile.exists()) {
      DriverStation.reportWarning("hood-config.json not found, using defaults", false);
      return;
    }

    try {
      ObjectMapper mapper = new ObjectMapper();
      Map<String, Double> data = mapper.readValue(m_configFile, LinkedHashMap.class);
      m_leftNearAngle  = data.getOrDefault("leftNearAngle", m_leftNearAngle);
      m_leftFarAngle   = data.getOrDefault("leftFarAngle", m_leftFarAngle);
      m_rightNearAngle = data.getOrDefault("rightNearAngle", m_rightNearAngle);
      m_rightFarAngle  = data.getOrDefault("rightFarAngle", m_rightFarAngle);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load hood-config.json: " + e.getMessage(), false);
    }
  }

  private void save() {
    try {
      ObjectMapper mapper = new ObjectMapper();
      mapper.enable(SerializationFeature.INDENT_OUTPUT);
      Map<String, Double> data = new LinkedHashMap<>();
      data.put("leftNearAngle", m_leftNearAngle);
      data.put("leftFarAngle", m_leftFarAngle);
      data.put("rightNearAngle", m_rightNearAngle);
      data.put("rightFarAngle", m_rightFarAngle);
      mapper.writeValue(m_configFile, data);
      DriverStation.reportWarning("Hood config saved to " + m_configFile.getPath(), false);
    } catch (IOException e) {
      DriverStation.reportError("Failed to save hood-config.json: " + e.getMessage(), false);
    }
  }
}
