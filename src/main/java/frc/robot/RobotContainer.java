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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.operatorConstants;
import frc.robot.commands.CommandFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pincer;
import frc.robot.subsystems.Lights;


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

    public final CommandXboxController driverJoystick = new CommandXboxController(0);
    public final CommandGenericHID pad = new CommandGenericHID(1);
    public final CommandXboxController operatorJoystick = new CommandXboxController(2);

    // Subsystem Instances
        public final Arm arm = new Arm();
        //public final Ascender ascender = new Ascender();
        //public final GroundAlgaePivot groundAlgaePivot = new GroundAlgaePivot();
        //public final GroundAlgaeWheels groundAlgaeWheels = new GroundAlgaeWheels();
        public final Pincer pincer = new Pincer();
 
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
        private final Command scorelL4CMD = commandFactory.scorelL4(false);

        //private final Command stowCMD = commandFactory.stow();
        private final Command feedCMD = commandFactory.feed();
        private final Command reefAlgaeHighCMD = commandFactory.reefAlgaeHigh();
        private final Command reefAlgaeLowCMD = commandFactory.reefAlgaeLow();
        private final Command netCMD = commandFactory.net();
        private final Command processorCMD = commandFactory.processor();
        private final Command groundAlgaeCMD = commandFactory.groundAlgae();

        // Custom Triggers
        Trigger LLHasTag = new Trigger(() -> drivetrain.LLHasTag());
        

    /* Path follower */
    //private final SendableChooser<Command> autoChooser;

    //public final Ascender ascender = new Ascender();

    public RobotContainer() {
        //autoChooser = AutoBuilder.buildAutoChooser("TEST");
        //SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        reefData.reset();

        CommandScheduler.getInstance().registerSubsystem(pincer);
        
        // Warmup path follower
        PathfindingCommand.warmupCommand().schedule();
        Timer.delay(3);
        FollowPathCommand.warmupCommand().schedule();
        Timer.delay(3);
        

        //CommandScheduler.getInstance().registerSubsystem(pad);
        //CommandScheduler.getInstance().registerSubsystem(arm);

        // Vision setup
        // Configure AprilTag detection
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", new int[]{6, 7, 8, 9, 10, 11}); // Only track these tag IDs
        }
        else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", new int[]{17, 18, 19, 20, 21, 22}); // Only track these tag IDs
        }
        else{
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", new int[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}); // Only track these tag IDs
        }
        LimelightHelpers.SetFiducialDownscalingOverride("limelight-front", 2.0f); // Process at half resolution for improved framerate and reduced range
    }

    private void configureBindings() {
        // Default commands
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                        // Drivetrain will execute this command periodically
                        drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed * elevator.getDrivetrainSpeedMultiplier().getAsDouble()) // Drive
                                                                                                                // forward with
                                                                                                                // negative Y
                                                                                                                // (forward)
                                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed * elevator.getDrivetrainSpeedMultiplier().getAsDouble()) // Drive left with negative X (left)
                                .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate * elevator.getDrivetrainSpeedMultiplier().getAsDouble()) // Drive counterclockwise with
                                                                                                // negative X (left)
                        ));
                elevator.setDefaultCommand(elevator.holdState());
                arm.setDefaultCommand(arm.holdState());
                pincer.setDefaultCommand(pincer.holdState());
                //ascender.setDefaultCommand(ascender.manualClimb(() -> operatorJoystick.getLeftY()));
        /*
        pad.button(1).onTrue(scorelL4CMD);
        pad.button(2).onTrue(scorelL3CMD);
        pad.button(3).onTrue(scorelL2CMD);
        pad.button(4).onTrue(scorelL1CMD);
        
        pad.button(5).onTrue(scorerL4CMD);
        pad.button(6).onTrue(scorerL3CMD);
        pad.button(7).onTrue(scorerL2CMD);
        pad.button(8).onTrue(scorerL1CMD);
  
         */
        //pad.button(20).onTrue(feedCMD);

        /*
        pad.button(22).and(() -> pincer.hasCoral()).onTrue(commandFactory.stow(true, false, elevator.isNearTop().getAsBoolean(), false));
        pad.button(22).and(() -> pincer.hasAlgae()).onTrue(commandFactory.stow(false, true, elevator.isNearTop().getAsBoolean(), false));
        pad.button(22).and(() -> (!pincer.hasCoral()) && (!pincer.hasAlgae())).onTrue(commandFactory.stow(false,false, elevator.isNearTop().getAsBoolean(), false));
         */
        
        pad.button(20).onTrue(new InstantCommand(() -> pincer.intake(),pincer));
        pad.button(21).onTrue(new InstantCommand(() -> pincer.stopIntake(),pincer));
        pad.button(28).onTrue(new InstantCommand(() -> pincer.exhaust(),pincer));
        pad.button(19).onTrue(new InstantCommand(()-> CommandScheduler.getInstance().cancel(netCMD,processorCMD,elevator.getCurrentCommand())));
        pad.button(27).onTrue(netCMD);
        //TODO: add climb stop and do it in code
        //pad.button(23).onTrue(new InstantCommand(() ->ascender.start(),ascender));
        //pad.button(24).onTrue(new InstantCommand(() ->ascender.stop(),ascender));
        pad.button(25).onTrue(reefAlgaeHighCMD);
        pad.button(26).onTrue(reefAlgaeLowCMD);
        //pad.button(29).onTrue(feedCMD);
        pad.button(30).onTrue(groundAlgaeCMD);

        //Keyad commands
        //Scoring Commands (left)
        pad.button(4).onTrue(commandFactory.scorelL1());
        pad.button(3).onTrue(commandFactory.scorelL2());
        pad.button(2).onTrue(commandFactory.scorelL3());
        pad.button(1)
                .and(()-> !arm.facingDownwards())
                        .onTrue(commandFactory.scorelL4(false));
        pad.button(1)
                .and(()-> arm.facingDownwards())
                        .onTrue(commandFactory.scorelL4(true));

        //Scoring Commands (right)
        pad.button(8).onTrue(commandFactory.scorelL1());
        pad.button(7).onTrue(commandFactory.scorelL2());
        pad.button(6).onTrue(commandFactory.scorelL3());
        pad.button(5)
                .and(()-> !arm.facingDownwards())
                        .onTrue(commandFactory.scorelL4(false));
        pad.button(5)
                .and(()-> arm.facingDownwards())
                        .onTrue(commandFactory.scorelL4(true));                        

        // Feed Commands
        pad.button(29).onTrue(commandFactory.feed());

                // Stow Commands
        pad.button(22)
                .and(elevator.isNearTop())
                .and(() -> !pincer.hasCoral())
                .and(() -> !pincer.hasAlgae())
                        .onTrue(commandFactory.stow(false, false, true, false));
        pad.button(22)
                .and(() ->pincer.hasCoral())
                .and(() -> !elevator.isLow().getAsBoolean())
                        .onTrue(commandFactory.stow(true, false, false, false));
        pad.button(22).and(() ->pincer.hasAlgae()).onTrue(commandFactory.stow(false, true, false, false));
        pad.button(22)
                .and(elevator.isLow())
                .and(() -> pincer.hasCoral())
                        .onTrue(commandFactory.stow(true, false, false, true));
        pad.button(22)
                .and(() -> !elevator.isNearTop().getAsBoolean())
                .and(() -> !pincer.hasCoral())
                .and(() -> !pincer.hasAlgae())
                        .onTrue(commandFactory.stow(false, false, false, false));

        // Small adjustments code
        driverJoystick.pov(90)
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.2))); //right
        driverJoystick.pov(270)
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.2))); //left
        driverJoystick.pov(0)
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.2).withVelocityY(0)));
        driverJoystick.pov(180)
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.2).withVelocityY(0)));


        // Driver joystick manual intake/exhaust wheels
        driverJoystick.leftBumper().whileTrue(pincer.manualIntake());
        driverJoystick.rightBumper().whileTrue(pincer.manualExhaust());

        // Limelight
        driverJoystick.start().and(LLHasTag).onTrue(Commands.runOnce(() -> drivetrain.resetToVision(true)));
        LLHasTag
                .onTrue(Commands.runOnce(() -> driverJoystick.setRumble(RumbleType.kBothRumble, 1)))
                .onFalse(Commands.runOnce(() -> driverJoystick.setRumble(RumbleType.kBothRumble, 0)));

        driverJoystick.a().onTrue(drivetrain.pathPIDTo(new Pose2d(0,1, new Rotation2d(0))));

        // Other drivebase code, could be used later?
/*
        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake)); // X-stance
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX())))); // idk
                                                                                                                // if
                                                                                                                // this
                                                                                                                // is
                                                                                                                // useful
*/
        // reset the field-centric heading on back press
        driverJoystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // Logging / Telemetry
        drivetrain.registerTelemetry(logger::telemeterize);


        // Operator Joystick Bindings
                //Scoring Commands
                operatorJoystick.a().onTrue(commandFactory.scorelL1());
                operatorJoystick.y().onTrue(commandFactory.scorelL2());
                operatorJoystick.rightBumper().onTrue(commandFactory.scorelL3());
                operatorJoystick.rightTrigger(operatorConstants.triggerBooleanThreshold)
                        .and(()-> !arm.facingDownwards())
                                .onTrue(commandFactory.scorelL4(false));
                operatorJoystick.rightTrigger(operatorConstants.triggerBooleanThreshold)
                        .and(()-> arm.facingDownwards())
                                .onTrue(commandFactory.scorelL4(true));
                
                // Feed Commands
                operatorJoystick.pov(0).onTrue(commandFactory.feed());

                // Stow Commands
                operatorJoystick.pov(90)
                        .and(elevator.isNearTop())
                        .and(() -> !pincer.hasCoral())
                        .and(() -> !pincer.hasAlgae())
                                .onTrue(commandFactory.stow(false, false, true, false));
                operatorJoystick.pov(90)
                        .and(() ->pincer.hasCoral())
                        .and(() -> !elevator.isLow().getAsBoolean())
                                .onTrue(commandFactory.stow(true, false, false, false));
                operatorJoystick.pov(90).and(() ->pincer.hasAlgae()).onTrue(commandFactory.stow(false, true, false, false));
                operatorJoystick.pov(90)
                        .and(elevator.isLow())
                        .and(() -> pincer.hasCoral())
                                .onTrue(commandFactory.stow(true, false, false, true));
                operatorJoystick.pov(90)
                        .and(() -> !elevator.isNearTop().getAsBoolean())
                        .and(() -> !pincer.hasCoral())
                        .and(() -> !pincer.hasAlgae())
                                .onTrue(commandFactory.stow(false, false, false, false));

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        //return autoChooser.getSelected();
        return null;
    }

    public void setMaxSpeed(double metersPerSecond) {
        MaxSpeed = metersPerSecond;
    }

}