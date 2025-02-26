// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.CommandFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundAlgaePivot;
import frc.robot.subsystems.GroundAlgaeWheels;
import frc.robot.subsystems.Pincer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Pincer;
import frc.robot.subsystems.keypad;


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
    private final CommandGenericHID pad = new CommandGenericHID(1);
    private final CommandXboxController operatorJoystick = new CommandXboxController(2);

    //private final Pincer pincer = new Pincer();

    

    
//intakes, climb, cancel
   

    

    

    // Subsystem Instances

    //public final Lights lights = new Lights();
    public final Arm arm = new Arm();
    //public final Ascender ascender = new Ascender();
    //public final GroundAlgaePivot groundAlgaePivot = new GroundAlgaePivot();
    //public final GroundAlgaeWheels groundAlgaeWheels = new GroundAlgaeWheels();
    public final Pincer pincer = null; //new Pincer();
 
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Elevator elevator = new Elevator();
        public final Lights lights = null;//new Lights();

        public final CommandFactory commandFactory = new CommandFactory(drivetrain, elevator, arm, pincer, lights);

        private final Command scorerL1CMD = commandFactory.scorerL1();
        private final Command scorerL2CMD = commandFactory.scorerL2();
        private final Command scorerL3CMD = commandFactory.scorerL3();
        private final Command scorerL4CMD = commandFactory.scorerL4();

        private final Command scorelL1CMD = commandFactory.scorelL1();
        private final Command scorelL2CMD = commandFactory.scorelL2();
        private final Command scorelL3CMD = commandFactory.scorelL3();
        private final Command scorelL4CMD = commandFactory.scorelL4();

        private final Command stowCMD = commandFactory.stow();
        private final Command feedCMD = commandFactory.feed();
        private final Command reefAlgaeHighCMD = commandFactory.reefAlgaeHigh();
        private final Command reefAlgaeLowCMD = commandFactory.reefAlgaeLow();
        private final Command netCMD = commandFactory.net();
        private final Command processorCMD = commandFactory.processor();
        

    /* Path follower */
    //private final SendableChooser<Command> autoChooser;

    //public final Ascender ascender = new Ascender();

    public RobotContainer() {
        //autoChooser = AutoBuilder.buildAutoChooser("TEST");
        //SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        reefData.reset();

        

        

        // Warmup path follower
        PathfindingCommand.warmupCommand().schedule();
        Timer.delay(3);
        FollowPathCommand.warmupCommand().schedule();
        Timer.delay(3);
        

        //CommandScheduler.getInstance().registerSubsystem(pad);
        //CommandScheduler.getInstance().registerSubsystem(arm);
    }

    private void configureBindings() {
        // Default commands
       elevator.setDefaultCommand(elevator.holdState());
       arm.setDefaultCommand(arm.holdState());


       // Default commands
       elevator.setDefaultCommand(elevator.holdState());
       arm.setDefaultCommand(arm.holdState());



       /*
        //scoreReefTrigger.onTrue(commandFactory.scoreL(pad.getReefL(),pad.getReef()));
        pad.button(1).onTrue(scorelL4CMD);
        pad.button(2).onTrue(scorelL3CMD);
        pad.button(3).onTrue(scorelL2CMD);
        pad.button(4).onTrue(scorelL1CMD);

        //pad.button(20).onTrue(feedCMD);
        pad.button(22).onTrue(stowCMD);
        pad.button(20).onTrue(new InstantCommand(() -> pincer.intake(),pincer));
        pad.button(21).onTrue(new InstantCommand(() -> pincer.stopIntake(),pincer));
        pad.button(28).onTrue(new InstantCommand(() -> pincer.exhaust(),pincer));
        pad.button(19).onTrue(new InstantCommand(()-> CommandScheduler.getInstance().cancel(netCMD,processorCMD,elevator.getCurrentCommand())));
        pad.button(27).onTrue(netCMD);
        //TODO: add climb stop and do it in code
        pad.button(23).onTrue(new InstantCommand(() ->ascender.start(),ascender));
        pad.button(24).onTrue(new InstantCommand(() ->ascender.stop(),ascender));
        pad.button(25).onTrue(reefAlgaeHighCMD);
        pad.button(26).onTrue(reefAlgaeLowCMD);
        pad.button(29).onTrue(feedCMD);
        pad.button(30).onTrue(groundAlgaeCMD);

         */

        //scoreReefTrigger.onTrue(scoreLCMD);
        //feedTrigger.onTrue(feedCMD);
        //stowTrigger.onTrue(stowCMD);
        //intakeStartTrigger.onTrue(new InstantCommand(() -> pincer.intake(),pincer));
        //intakeStopTrigger.onTrue(new InstantCommand(() -> pincer.stopIntake(),pincer));
        //intakeExhaustTrigger.onTrue(new InstantCommand(() -> pincer.exhaust(),pincer));
        //cancelScoreTrigger.onTrue(new InstantCommand(()-> CommandScheduler.getInstance().cancel(netCMD,processorCMD,scoreLCMD)));
        //netTrigger.onTrue(netCMD);
        //TODO: add climb stop and do it in code
        //climbTrigger.onTrue(new InstantCommand(() ->ascender.climb()),ascender);
        //climbCancelTrigger.onTrue(new InstantCommand() ->ascender.st)
        //reefAlgaeHighTrigger.onTrue(reefAlgaeHighCMD);
        //reefAlgaeLowTrigger.onTrue(reefAlgaeLowCMD);

        driverJoystick.a().onTrue(elevator.goToFeed());
        driverJoystick.b().onTrue(elevator.goToL3());
        driverJoystick.x().onTrue(elevator.goToStow(() -> arm.facingDownwards()));        

       //driverJoystick.a().onTrue(elevator.goToL(Constants.reef.reefLs.lL3,1));
       //driverJoystick.b().onTrue(arm.pivotToL2L3());
       //driverJoystick.a().onTrue(arm.pivotToFeed());
       //driverJoystick.a().onTrue(scoreL2CMD);
       //driverJoystick.b().onTrue(scoreL3CMD);

       /*


         */

        // driverJoystick.b().onTrue(elevator.goToL2());
        // driverJoystick.x().onTrue(elevator.goToL3());
        // driverJoystick.y().onTrue(elevator.goToL4());

        // operatorJoystick.a().onTrue(arm.pivotToFeed());


        /*
        arm.setDefaultCommand(arm.holdState());
        pincer.setDefaultCommand(pincer.holdState());

        // Manual cycling testing code

        driverJoystick.a().onTrue(elevator.goToL3());
        driverJoystick.b().onTrue(arm.pivotToL2L3());

        driverJoystick.x().onTrue(elevator.goToFeed());
        driverJoystick.y().onTrue(arm.pivotToFeed());

        
        driverJoystick.leftBumper().onTrue(pincer.intake());
        driverJoystick.leftBumper().onFalse(pincer.stopIntake());
        driverJoystick.rightBumper().onTrue(pincer.exhaust());
        driverJoystick.rightBumper().onFalse(pincer.stopIntake());

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        */
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
                 
        // Other drivebase code, could be used later?
/*
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
        /*
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
*/
        // reset the field-centric heading on back press
        driverJoystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // reset the pose
        //driverJoystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.resetToVision(true)));

        // ???
        // drivetrain.registerTelemetry(logger::telemeterize);

        //ascender.setDefaultCommand(ascender.manualClimb(() -> operatorJoystick.getLeftY()));
        arm.setDefaultCommand(arm.holdState());
        operatorJoystick.a().onTrue(arm.pivotToFeed());
        operatorJoystick.b().onTrue(arm.pivotToL2L3());
    }

    // Command compositions (there is probably a better place for these)
    // Missing: limelighfunctionality
/*
    public Command climbSequence() {
            return elevator.goToStow().andThen(ascender.climb()); // climb() returns null for now
    }
*/
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
                            pincer.stopIntake()//,
                            //elevator.goToStow(),
                            //arm.stowPivot()

            );
    }
    /*
    public Command processor() {
            return new SequentialCommandGroup(
                            groundAlgaePivot.goToScore(),
                            groundAlgaeWheels.outtakeAlgae(),
                            groundAlgaePivot.goToStow());
    }

    public Command reefAlgae() {
            return null;
    }
  */
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        //return autoChooser.getSelected();
        return null;
    }
}