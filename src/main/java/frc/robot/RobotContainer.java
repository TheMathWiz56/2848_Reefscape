// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants.operatorConstants;
import frc.robot.Constants.reef.reefLs;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pincer;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Ascender;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private double speedMultiplier = 1.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 2% deadband //changed from 10%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric preciseAdjustments = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController driverJoystick = new CommandXboxController(0);
    public final CommandXboxController operatorJoystick = new CommandXboxController(2);

    // Subsystem Instances
        public final Arm arm = new Arm();
        public final Ascender ascender = new Ascender();
        public final Pincer pincer = new Pincer();
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Elevator elevator = new Elevator();
        public final Lights lights = null;//new Lights();
        
        // Command Factory
        public final CommandFactory commandFactory = new CommandFactory(drivetrain, elevator, arm, pincer, lights);

        // Custom Triggers
        Trigger LLHasTag = new Trigger(() -> drivetrain.LLHasTag());    
        private final BooleanSupplier manualDrivebase = () -> Math.hypot(driverJoystick.getLeftX(), driverJoystick.getLeftY()) > 0.25
                                                                || Math.abs(driverJoystick.getRightX()) > 0.25;


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // NamedCommands.registerCommand("GoTo_L4", commandFactory.goTolL4());
        // NamedCommands.registerCommand("Exhaust_Coral", commandFactory.exhaustCoral());
        // NamedCommands.registerCommand("Score_L4", commandFactory.scorelL4(true));
        // NamedCommands.registerCommand("Score_L2", commandFactory.scorelL2());
        // NamedCommands.registerCommand("Stow_Empty", commandFactory.stow(false, false, true, false));
        // NamedCommands.registerCommand("Stow_Coral", commandFactory.stow(true, false, false, true));
        // NamedCommands.registerCommand("Reset_To_Vision", Commands.runOnce(() -> drivetrain.resetToVision(true)));
        // NamedCommands.registerCommand("Align_Right", drivetrain.pathPIDToTagRightSelect());
        // NamedCommands.registerCommand("Reef_Stall", Commands.run(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.3)), drivetrain));
        // NamedCommands.registerCommand("Feed", commandFactory.feedSequential());
        // NamedCommands.registerCommand("Pause_For_Feed", new WaitCommand(15).until(() -> pincer.hasCoral()));

        // new EventTrigger("Feed_Event").onTrue(commandFactory.feedSequential());
        // new EventTrigger("Stow_Empty_Event").onTrue(commandFactory.stow(false, false, true, false));
        // new EventTrigger("Stow_Coral_Event").onTrue(commandFactory.stow(true, false, false, true));
        // new EventTrigger("GoTo_L4_Event").onTrue(commandFactory.goTolL4());
        // new EventTrigger("Exhaust_Coral_Event").onTrue(commandFactory.exhaustCoral());

        autoChooser = AutoBuilder.buildAutoChooser("Center_Left");
        autoChooser.addOption("Right_2_Coral", new PathPlannerAuto("Left_2_Coral", true));
        autoChooser.addOption("Right_1_Coral", new PathPlannerAuto("Left_1_Coral", true));
        autoChooser.addOption("Center_Right", new PathPlannerAuto("Center_Left", true));

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
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                        // Drivetrain will execute this command periodically
                        drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed * speedMultiplier * elevator.getDrivetrainSpeedMultiplier().getAsDouble()) // Drive
                                                                                                                // forward with
                                                                                                                // negative Y
                                                                                                                // (forward)
                                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed * speedMultiplier * elevator.getDrivetrainSpeedMultiplier().getAsDouble()) // Drive left with negative X (left)
                                .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate * speedMultiplier * elevator.getDrivetrainSpeedMultiplier().getAsDouble()) // Drive counterclockwise with
                                .withDeadband(MaxSpeed * 0.1 * elevator.getDrivetrainSpeedMultiplier().getAsDouble())
                                .withRotationalDeadband(MaxAngularRate * 0.1 * elevator.getDrivetrainSpeedMultiplier().getAsDouble())                                                            // negative X (left)
                        ));
                elevator.setDefaultCommand(elevator.holdState());
                arm.setDefaultCommand(arm.holdState());
                pincer.setDefaultCommand(pincer.holdState());
                ascender.setDefaultCommand(ascender.manualClimb(() -> operatorJoystick.getLeftY()));

        // Drivebase Telemetry
        drivetrain.registerTelemetry(logger::telemeterize);

        // Drive Joystick Bindings
                // Small adjustments code
                driverJoystick.pov(90)
                        .whileTrue(drivetrain.applyRequest(() -> preciseAdjustments.withVelocityX(0).withVelocityY(-0.125))); //right
                driverJoystick.pov(270)
                        .whileTrue(drivetrain.applyRequest(() -> preciseAdjustments.withVelocityX(0).withVelocityY(0.125))); //left
                driverJoystick.pov(0)
                        .whileTrue(drivetrain.applyRequest(() -> preciseAdjustments.withVelocityX(0.125).withVelocityY(0)));
                driverJoystick.pov(180)
                        .whileTrue(drivetrain.applyRequest(() -> preciseAdjustments.withVelocityX(-0.125).withVelocityY(0)));

                // Driver joystick manual intake/exhaust wheels
                driverJoystick.leftBumper().whileTrue(pincer.manualIntake());
                driverJoystick.rightBumper().whileTrue(pincer.manualExhaust());

                // reset the field-centric pose to vision pose
                driverJoystick.start().and(LLHasTag).onTrue(Commands.runOnce(() -> drivetrain.resetToVision(true)));
                // reset the field-centric heading on back press
                driverJoystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                LLHasTag.onTrue(Commands.runOnce(() -> driverJoystick.setRumble(RumbleType.kBothRumble, 1)))
                        .onFalse(Commands.runOnce(() -> driverJoystick.setRumble(RumbleType.kBothRumble, 0)));

                // Processor
                driverJoystick.a().onTrue(commandFactory.processor());

                driverJoystick.rightTrigger(operatorConstants.triggerBooleanThreshold).onTrue(arm.reefAlgaeHigh2nd());

                //  !pincer.hasCoral() || 
                driverJoystick.x().and(LLHasTag).onTrue(commandFactory.autoReefCoralLeft().until(() -> manualDrivebase.getAsBoolean()));
                driverJoystick.b().and(LLHasTag).onTrue(commandFactory.autoReefCoralRight().until(() -> manualDrivebase.getAsBoolean()));
                driverJoystick.leftTrigger(operatorConstants.triggerBooleanThreshold)
                        .and(LLHasTag)
                                .onTrue(commandFactory.autoReefAlgae()
                                .until(() -> manualDrivebase.getAsBoolean()))
                                ;
                // Make elevator go up a set amount - very not working
                // driverJoystick.y().onTrue(elevator.goUpByDistance(1.25));

                // Zero elevator manually
                driverJoystick.y().onTrue(elevator.autoZeroEncoder());

                // A is being used for processor right now so this should be changed
                // driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake)); // X-stance

        
        // Operator Joystick Bindings
                //Scoring Commands
                operatorJoystick.y().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).negate()).onTrue(commandFactory.scoreLevel(reefLs.L2));
                operatorJoystick.rightBumper().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).negate()).onTrue(commandFactory.scoreLevel(reefLs.L3));
                operatorJoystick.rightTrigger(operatorConstants.triggerBooleanThreshold).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).negate())
                        .and(()-> !arm.facingDownwards())
                                .onTrue(commandFactory.scoreLevel(reefLs.L4, false));
                operatorJoystick.rightTrigger(operatorConstants.triggerBooleanThreshold).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).negate())
                        .and(()-> arm.facingDownwards())
                                .onTrue(commandFactory.scoreLevel(reefLs.L4, true));
                
                // Feed Commands
                operatorJoystick.pov(0).onTrue(commandFactory.feed());

                // Stow Commands

                //empty near top
                operatorJoystick.pov(90)
                        .and(elevator.isNearTop())
                        .and(() -> !pincer.hasCoral())
                        .and(() -> !pincer.hasAlgae())
                                .onTrue(commandFactory.stow(false, false, true, false));
                //coral not low
                operatorJoystick.pov(90)
                        .and(() ->pincer.hasCoral())
                        .and(() -> !elevator.isLow().getAsBoolean())
                                .onTrue(commandFactory.stow(true, false, false, false));
                //all algae
                operatorJoystick.pov(90).and(() ->pincer.hasAlgae()).onTrue(commandFactory.stow(false, true, false, false));
                //low coral
                operatorJoystick.pov(90)
                        .and(elevator.isLow())
                        .and(() -> pincer.hasCoral())
                                .onTrue(commandFactory.stow(true, false, false, true));
                //not high empty
                operatorJoystick.pov(90)
                        .and(() -> !elevator.isNearTop().getAsBoolean() && !elevator.isLow().getAsBoolean())
                        .and(() -> !pincer.hasCoral())
                        .and(() -> !pincer.hasAlgae())
                                .onTrue(commandFactory.stow(false, false, false, false));

                operatorJoystick.pov(90)
                        .and(elevator.isLow())
                        .and(() -> !pincer.hasCoral())
                        .and(() -> !pincer.hasAlgae())
                                .onTrue(commandFactory.stow(false, false, false, true));

                
                operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).and(operatorJoystick.y().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising))
                        .onTrue(elevator.setLevelQueue(2));
                operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).and(operatorJoystick.rightBumper().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising))
                        .onTrue(elevator.setLevelQueue(3));
                operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).and(operatorJoystick.rightTrigger(operatorConstants.triggerBooleanThreshold).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising))
                        .onTrue(elevator.setLevelQueue(4));


                //Is High
                operatorJoystick.a()
                        .and(elevator.isNearTop())
                        .and(() -> !pincer.hasAlgae())
                                .onTrue(commandFactory.stow(true, false, true, false));
                //Is Middle
                operatorJoystick.a()
                        .and(() -> !elevator.isLow().getAsBoolean())
                        .and(() -> !elevator.isNearTop().getAsBoolean())
                        .and(() -> !pincer.hasAlgae())
                                .onTrue(commandFactory.stow(true, false, false, false));
                //Is Low
                operatorJoystick.a()
                        .and(elevator.isLow())
                        .and(() -> !pincer.hasAlgae())
                                .onTrue(commandFactory.stow(true, false, false, true));    
                                
                                
                operatorJoystick.start().onTrue(commandFactory.stow(false, true, false, false));
                

                //reef algae

                operatorJoystick.leftBumper().onTrue(
                        commandFactory.reefAlgaeHigh()
                );

                operatorJoystick.leftTrigger(operatorConstants.triggerBooleanThreshold).onTrue(
                        commandFactory.reefAlgaeLow()
                );
                

                /*
                accounted for i think

                low coral
                high coral - normal
                normal - normal

                low  algae - normal 
                normal algae - normal 
                high algae - normal 

                low empty - normal
                normal empty - normal
                high empty - high empty
                */                
                

        //Pincer
                operatorJoystick.b().onTrue(pincer.pincerAlgaeHold());
                operatorJoystick.rightStick().onTrue(pincer.pincerFunnel2());
                operatorJoystick.x().onTrue(pincer.pincerFunnel());

        operatorJoystick.pov(270).onTrue(commandFactory.net());

        operatorJoystick.pov(180).onTrue(commandFactory.groundAlgae());
        
        //operatorJoystick.y().toggleOnTrue(pincer.holdIntakeCmd()); */


    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}




class CommandFactory{
    private final CommandSwerveDrivetrain drive;
    private final Elevator elevator;
    private final Arm arm;
    private final Pincer pincer;
    private final Lights lights;

    public CommandFactory(CommandSwerveDrivetrain drive, Elevator elevator, Arm arm, Pincer pincer, Lights lights){
        this.drive = drive;
        this.elevator = elevator;
        this.arm = arm;
        this.pincer = pincer;
        this.lights = lights;
    }

/*Moves only elevator, pivot and intake to score on reef */
  
    // Score a level.
    public Command scoreLevel(Constants.reef.reefLs level, boolean facingDownwards) {
        return new ConditionalCommand(Commands.none(), arm.goStraightOn(), () -> facingDownwards)
            .andThen(elevator.goToL(level))
            .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
                Constants.reef.reefToState.get(level)
            )))
            .andThen(pincer.exhaust().until(() -> !pincer.hasCoral())) // In some of the cmds this would be .andThen(pincer.exhaust()).andThen(pincer.holdState()).until(() -> !pincer.hasCoral()), this seems to be better though
            .andThen(pincer.stopIntake()).unless(() -> !pincer.hasCoral());
    }

    // Score a level.
    public Command scoreLevel(Constants.reef.reefLs level) {
        return scoreLevel(level, true);
    }

    // This L1 code is different than scoreLevel(), but it probably would work, we aren't using it either way rn though
    // public Command scoreL1(){
    //     return elevator.goToL(Constants.reef.reefLs.L1)
    //     .andThen(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
    //         Constants.reef.reefToState.get(
    //             Constants.reef.reefLs.L1
    //         )
    //     )))
    //     .andThen(pincer.exhaust())
    //      .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
    //     .finallyDo((interrupted) ->{
    //           pincer.stopIntake();
    //         });
    // }

    public Command goTolL4(){
        return elevator.goToL(Constants.reef.reefLs.L4)
                .alongWith(arm.moveToPoint(Constants.ArmConstants.setPoints.get(
                    Constants.reef.reefToState.get(
                        Constants.reef.reefLs.L4
                    )
                )));
    }

    public Command exhaustCoral(){
        return pincer.exhaust()
                .andThen(pincer.holdState());
    }

/*stows. Uses sensor to determine which stow */
    public Command stow(boolean hasCoral, boolean hasAlgae, boolean isNearTop, boolean IsLow){
        Command output;
        if(hasCoral && !IsLow){ // coral not low
            output = pincer.stopIntake()
            .andThen(new ParallelCommandGroup(arm.coralStow(),
            elevator.coralStow())
            ).andThen(pincer.pincerFunnel());
        }
        else if(hasCoral && IsLow){ // coral low
            output = pincer.stopIntake()
            .andThen(elevator.coralStow())
            .andThen(arm.coralStow())
            .andThen(pincer.pincerFunnel());
        }
        else if(hasAlgae){ // no coral -- algae or low
            output = pincer.stopIntake()
            .andThen(new ParallelCommandGroup(arm.algaeStow(),
            elevator.algaeStow()));
        }
        else if(isNearTop) {
            output =  pincer.stopIntake()
                .andThen(new ParallelCommandGroup(arm.goStraightOn()
                , elevator.emptyStow()))
                .andThen(arm.emptyStow())
                .andThen(pincer.pincerFunnel());
        }
        else if (IsLow){
            output = pincer.stopIntake()
            .andThen(arm.emptyStow())
            .andThen(elevator.emptyStow())
            .andThen(pincer.pincerFunnel());
        }
        else{
            output = pincer.stopIntake()
            .andThen(new ParallelCommandGroup(arm.emptyStow()
            , elevator.emptyStow()))
            .andThen(pincer.pincerFunnel());
            }

        return output;
    }
    /*move claw, pivot, elevator to intake */
    public Command feed(){
        return //pincer.pincerFunnel()
        new ParallelCommandGroup(elevator.goToFeed(),
         arm.pivotToFeed())
         .andThen(pincer.pincerFunnel())
         .andThen(pincer.intake())
         .andThen(pincer.holdState().until(()->pincer.hasCoral()))
         .andThen(pincer.stopIntake());
    }

    public Command feedSequential(){
        return elevator.goToFeed()
            .andThen(arm.pivotToFeed())
            .andThen(pincer.pincerFunnel())
            .andThen(pincer.intake())
            .andThen(pincer.holdState().until(()->pincer.hasCoral()))
            .andThen(pincer.stopIntake());
    }

    public Command reefAlgaeHigh(){
            return //pincer.pincerAlgae()
            elevator.reefAlgaeHigh()
            .andThen(arm.reefAlgaeHigh())
            //.andThen(pincer.reefAlgae())
            .andThen(pincer.intake())
            //.andThen(new WaitUntilCommand(()->pincer.hasAlgae()))
            .andThen(pincer.pincerAlgaeHold());
            //.andThen(Commands.waitUntil(() -> pincer.hasAlgae()))
            //.andThen(arm.pivotToParallel());
            //.until(() -> pincer.hasAlgae())
            //.finallyDo((interrupted) -> pincer.stopIntake());
        }

    private Command reefAlgaeHighNoPinch(){
        return elevator.reefAlgaeHighAuto()
            .andThen(arm.reefAlgaeHigh());
    }

    private Command reefAlgaeLowNoPinch(){
        return elevator.reefAlgaeLowAuto()
            .andThen(arm.reefAlgaeLow());
    }
        
    public Command reefAlgaeLow(){
        
            return //pincer.pincerAlgae()
            elevator.reefAlgaeLow()
            .andThen(arm.reefAlgaeLow())
            //andThen(pincer.reefAlgae())
            .andThen(pincer.intake())
            //.andThen(new WaitUntilCommand(()->pincer.hasAlgae()))
            .andThen(pincer.pincerAlgaeHold());
            //.andThen(Commands.waitUntil(() -> pincer.hasAlgae()))
            //.andThen(arm.pivotToParallel());
            //.until(() -> pincer.hasAlgae())
            //.finallyDo((interrupted) -> pincer.stopIntake());
    }

/*score net net */
    public Command net(){
        return new ParallelCommandGroup(elevator.goToNet(),
        arm.goToNet())
        .andThen(pincer.exhaust())
        .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
        .finallyDo((interrupted) ->{pincer.stopIntake(); pincer.pincerFunnel();});
    }
    /*score processor */
    public Command processor(){
        /* return elevator.goToProcessor()
        .andThen(arm.goToProcessor())
        
        .andThen(pincer.exhaust())
        .finallyDo((interrupted) ->{
            pincer.stopIntake();
          }); */

        return new ParallelCommandGroup(elevator.goToProcessor(),
        arm.goToProcessor())
        .andThen(pincer.exhaust())
        .andThen(new WaitCommand(Constants.PincerConstants.scoreIntakeDelay))
        .finallyDo((interrupted) ->
              {pincer.stopIntake(); pincer.pincerFunnel();});  
    }
    public Command groundAlgae(){
        return new ParallelCommandGroup(elevator.goToGroundAlgae(),
        arm.goToGroundAlgae());
    }

    private Command pinceAlgae(){
        return pincer.intake()
            .andThen(pincer.pincerAlgaeHold());
    }

    private Command autoReefAlgaeStow(){
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(0, stow(true, false, false, true)),
                Map.entry(1, stow(true, false, false, false)),
                Map.entry(2, stow(true, false, true, false))
            )
            , () -> elevator.getState());
    }

    private Command autoReefAlgaeHigh(){
        return drive.pathPIDToTagMiddleSelect()
                .alongWith(autoReefAlgaeStow())
            .andThen(reefAlgaeHighNoPinch()
                .raceWith(Commands.run(() -> drive.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.45)), drive)))
            .andThen(pinceAlgae())
            .andThen(Commands.run(() -> drive.setControl(new SwerveRequest.RobotCentric().withVelocityX(-1)), drive)
                .withTimeout(.5)
                .andThen(arm.reefAlgaeHigh2nd()))
            ;
    }

    private Command autoReefAlgaeLow(){
        return drive.pathPIDToTagMiddleSelect()
                .alongWith(autoReefAlgaeStow())
            .andThen(reefAlgaeLowNoPinch()
                .raceWith(Commands.run(() -> drive.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.45)), drive)))
            .andThen(pinceAlgae())
            .andThen(Commands.run(() -> drive.setControl(new SwerveRequest.RobotCentric().withVelocityX(-1)), drive)
                .withTimeout(.5)
                .andThen(arm.reefAlgaeHigh2nd()))
            ;
    }

    public Command autoReefAlgae(){
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(17, autoReefAlgaeLow()),
                Map.entry(18, autoReefAlgaeHigh()),
                Map.entry(19, autoReefAlgaeLow()),
                Map.entry(20, autoReefAlgaeHigh()),
                Map.entry(21, autoReefAlgaeLow()),
                Map.entry(22, autoReefAlgaeHigh()), 
                Map.entry(6, autoReefAlgaeLow()),
                Map.entry(7, autoReefAlgaeHigh()),
                Map.entry(8, autoReefAlgaeLow()),
                Map.entry(9, autoReefAlgaeHigh()),
                Map.entry(10, autoReefAlgaeLow()),
                Map.entry(11, autoReefAlgaeHigh()))
        , () -> drive.getTag());
    }


    public Command autoReefCoralLeft(){
        return drive.pathPIDToTagLeftSelect()
            .andThen(Commands.run(() -> drive.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.45)), drive)
            .raceWith(
                new SelectCommand<>(
                    Map.ofEntries(
                        Map.entry(2, scoreLevel(reefLs.L2)),
                        Map.entry(3, scoreLevel(reefLs.L3)),
                        Map.entry(4, scoreLevel(reefLs.L4, true)))
                    , () -> elevator.getLevelQueue())));
    }

    public Command autoReefCoralRight(){
        return drive.pathPIDToTagRightSelect()
            .andThen(Commands.run(() -> drive.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.45)), drive)
            .raceWith(
                new SelectCommand<>(
                    Map.ofEntries(
                        Map.entry(2, scoreLevel(reefLs.L2)),
                        Map.entry(3, scoreLevel(reefLs.L3)),
                        Map.entry(4, scoreLevel(reefLs.L4, true)))
                    , () -> elevator.getLevelQueue())));
    }

}
