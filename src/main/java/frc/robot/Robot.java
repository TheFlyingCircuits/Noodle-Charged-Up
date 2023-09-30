// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LoggerMode;


public class Robot extends LoggedRobot {
  //turn this true to use the simulator!!
  public LoggerMode robotMode = LoggerMode.ROBOT_REAL;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private String previousPath = "";

  @Override
  public void robotInit() {
    System.out.println("[Init] Creating " + this.getClass().getName());
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    System.out.println("Ghost Buster 10 second wait started!");
    if (isReal()) Timer.delay(10); // Ghost Busters! Make sure everything has time to power up before initializing

    Logger.getInstance().recordMetadata("projectName", "Noodle-Charged-Up");

    switch (robotMode) {
      case ROBOT_SIM:
        Logger.getInstance().addDataReceiver(new NT4Publisher());
        break;
      case ROBOT_REAL:
        Logger.getInstance().addDataReceiver(new NT4Publisher());
        break;
      case ROBOT_REAL_LOGGED:
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/home/lvuser/deploy/logs")); 
        Logger.getInstance().addDataReceiver(new NT4Publisher());
        break;
      case ROBOT_REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath));
        break;
    }

    Logger.getInstance().start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.arm.setToBrakeMode(false);
  }

  @Override
  public void disabledPeriodic() {
    String path = m_robotContainer.autoChooser.getSelected();
    if (path == null || previousPath.equals(path)) return;

    m_robotContainer.pathGroup = PathPlanner.loadPathGroup(
      path, PathPlanner.getConstraintsFromPath(path));
    previousPath = path;
  }

  @Override
  public void disabledExit() {
    m_robotContainer.arm.setToBrakeMode(true);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
