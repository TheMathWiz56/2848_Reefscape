// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundAlgaePivot;
import frc.robot.subsystems.GroundAlgaeWheels;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Pincer;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private CommandGenericHID operatorKeypad = new CommandGenericHID(1);
    private final CommandXboxController operatorJoystick = new CommandXboxController(2);

    // Subsystem Instances
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Lights lights = new Lights();
    public final Elevator elevator = new Elevator();
    public final Arm arm = new Arm();
    public final Ascender ascender = new Ascender(); // Note to self: Ascender class hasn't been started yet
    public final GroundAlgaePivot groundAlgaePivot = new GroundAlgaePivot();
    public final GroundAlgaeWheels groundAlgaeWheels = new GroundAlgaeWheels();
    public final Pincer pincer = new Pincer();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("TEST");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup path follower
        PathfindingCommand.warmupCommand().schedule();
        Timer.delay(3);
        FollowPathCommand.warmupCommand().schedule();
        Timer.delay(3);
    }

    private void configureBindings() {
        // Default commands
        elevator.setDefaultCommand(elevator.holdState());
        arm.setDefaultCommand(arm.holdState());
        pincer.setDefaultCommand(pincer.holdState());
        groundAlgaePivot.setDefaultCommand(groundAlgaePivot.holdState());
        groundAlgaeWheels.setDefaultCommand(groundAlgaeWheels.holdState());

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                          // negative X (left)
                ));

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake)); // X-stance
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX())))); // idk
                                                                                                                // if
                                                                                                                // this
                                                                                                                // is
                                                                                                                // useful

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        driverJoystick.pov(0)
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driverJoystick.pov(180)
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        /*
         * Run SysId routines when holding back/start and X/Y.
         * Note that each routine should be run exactly once in a single log.
         * Disable for competition
         */
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // reset the pose
        driverJoystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.resetToVision(true)));

        // testing
        driverJoystick.x()
                .onTrue(lights.inAction())
                .onFalse(lights.actionComplete());

        drivetrain.registerTelemetry(logger::telemeterize);

        // Trigger to zero motor voltage on elevator if limit switches trip
        // I think this will cause a problem where if the elevator goes too far down, it
        // will not be able to go back up again?
        Trigger elevatorLimitTrigger = new Trigger(elevator::getLimitSwitches).whileTrue(elevator.elevatorAtHardLimit());



    }

    // Command compositions (there is probably a better place for these)
    // Missing: limelight functionality

    public Command climbSequence() {
            return elevator.goToStow().andThen(ascender.climb()); // climb() returns null for now
    }

    public Command scoreLevel(int level) {
            Command armPivot, elevatorPivot;
            switch (level) {
                    case 1:
                            armPivot = arm.pivotToL1();
                            elevatorPivot = elevator.goToL1();
                            break;

                    case 2:
                            armPivot = arm.pivotToL2L3();
                            elevatorPivot = elevator.goToL2();
                            break;

                    case 3:
                            armPivot = arm.pivotToL2L3();
                            elevatorPivot = elevator.goToL3();
                            break;

                    case 4:
                            armPivot = arm.pivotToL4();
                            elevatorPivot = elevator.goToL4();
                            break;

                    default:
                            return Commands.none();
            }

            return new SequentialCommandGroup(
                            elevatorPivot,
                            armPivot,
                            pincer.exhaust(),
                            Commands.waitSeconds(0.3),
                            pincer.stopIntake(),
                            elevator.goToStow(),
                            arm.stowPivot()

            );
    }

    public Command processor() {
            return new SequentialCommandGroup(
                            groundAlgaePivot.goToScore(),
                            groundAlgaeWheels.outtakeAlgae(),
                            groundAlgaePivot.goToStow());
    }

    public Command reefAlgae() {
            return null;
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}