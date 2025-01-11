// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  //private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  //private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  //private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Arm arm = new Arm(); // My arm

  /*private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();*/

  //private final Telemetry logger = new Telemetry(MaxSpeed);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private void configureBindings() {

    m_chooser.setDefaultOption("None", Commands.idle(arm));
    m_chooser.addOption("Q_Forward", arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_chooser.addOption("Q_Reverse", arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_chooser.addOption("D_Forward", arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_chooser.addOption("D_Reverse", arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData(m_chooser);
    /*drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));*/
    arm.setDefaultCommand(Commands.idle(arm)); // All control handled in periodic

    /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));*/

    // reset the field-centric heading on left bumper press
    /*joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative())
    .andThen(drivetrain.runOnce(()->drivetrain.seedFieldRelative(new Pose2d(drivetrain.getState().Pose.getTranslation(), new Rotation2d())))));*/

    /*joystick.leftBumper().onTrue(drivetrain.runOnce(()->drivetrain.seedFieldRelative(new Pose2d(drivetrain.getState().Pose.getTranslation(), new Rotation2d()))));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);*/
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
