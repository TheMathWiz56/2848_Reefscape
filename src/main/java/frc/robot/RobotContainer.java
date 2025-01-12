// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  private final Arm arm = new Arm();
  // private final Trigger updated_reference = new Trigger(()->arm.is_reference_updated());
  private CommandGenericHID keypad = new CommandGenericHID(0);
  
  

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private void configureBindings() {
    // updated_reference.onTrue(arm.go_to_reference()); Disabled for manual testing
    arm.setDefaultCommand(arm.hold_position());
    keypad.button(1).onTrue(arm.go_to_reference(.1));
    keypad.button(1).onFalse(arm.go_to_reference(.2));
    m_chooser.setDefaultOption("None", Commands.idle(arm));
    m_chooser.addOption("Q_Forward", arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_chooser.addOption("Q_Reverse", arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_chooser.addOption("D_Forward", arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_chooser.addOption("D_Reverse", arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData(m_chooser);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
