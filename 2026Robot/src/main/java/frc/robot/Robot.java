package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Robot lifecycle class with AdvantageScope logging and motor brake management.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final Timer disabledTimer = new Timer();

  public Robot() {
    // Start data logging for AdvantageScope replay analysis
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer();
  }

  /**
   * Called periodically (every 20ms) regardless of mode.
   * Runs the CommandScheduler to execute all scheduled commands.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * Called once when the robot enters disabled mode.
   * Starts the brake-to-coast transition timer.
   */
  @Override
  public void disabledInit() {
    disabledTimer.reset();
    disabledTimer.start();
  }

  /**
   * Called periodically while disabled.
   * After WHEEL_LOCK_TIME seconds, switches motors from brake to coast mode.
   */
  @Override
  public void disabledPeriodic() {
    // Hold brake mode for WHEEL_LOCK_TIME after disabling, then release to coast
    // so the robot can be pushed freely in the pits
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * Called once when exiting disabled mode.
   */
  @Override
  public void disabledExit() {
  }

  /**
   * Called once at the start of autonomous mode.
   * Enables motor brakes and schedules the selected autonomous command.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /**
   * Called periodically during autonomous mode.
   * Command execution is handled in robotPeriodic().
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * Called once when exiting autonomous mode.
   */
  @Override
  public void autonomousExit() {
  }

  /**
   * Called once at the start of teleop mode.
   * Enables motor brakes and cancels any running autonomous commands.
   */
  @Override
  public void teleopInit() {
    m_robotContainer.setMotorBrake(true);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * Called periodically during teleop mode.
   * Command execution is handled in robotPeriodic().
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * Called once when exiting teleop mode.
   */
  @Override
  public void teleopExit() {
  }

  /**
   * Called once at the start of test mode.
   * Cancels all running commands to prepare for test routines (like SysId).
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * Called periodically during test mode.
   * Used for characterization and hardware diagnostics.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Called once when exiting test mode.
   */
  @Override
  public void testExit() {
  }
}
