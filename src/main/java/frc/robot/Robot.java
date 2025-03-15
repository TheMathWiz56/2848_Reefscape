// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private double autonomous_delay;

  public Robot() {
    m_robotContainer = new RobotContainer();

    // Publishes autonomous delay key to smart dashboard
    SmartDashboard.putNumber("Autonomous Delay", 0);

    // Set the logger to log to the first flashdrive plugged in. This should be turned back on once a USB drive is plugged into the roboRIO. Good for debugging
    //SignalLogger.setPath("/media/sda1/");
    SignalLogger.stop(); // remove to re-enable internal logging
    // DataLogManager.start(); // /logs folder in sda1   // Logs Network Table information
    // SignalLogger.start();

    CanBridge.runTCP();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    m_robotContainer.driverJoystick.setRumble(RumbleType.kBothRumble, 0.0);
  }

  @Override
  public void disabledPeriodic() {
    autonomous_delay = SmartDashboard.getNumber("Autonomous Delay", 0);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.drivetrain.useMegaTag2(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
        Timer.delay(SmartDashboard.getNumber("Autonomous Delay", 0));
        m_autonomousCommand.schedule();
    }

    // Elastic.selectTab("Autonomous"); // Causing lag, I think. If turned back on might cause commandSchedulerLoop issues
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

    m_robotContainer.drivetrain.useMegaTag2(true);

    // Elastic.selectTab("Teleoperated"); // Causing lag, I think. If turned back on might cause commandSchedulerLoop issues
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
