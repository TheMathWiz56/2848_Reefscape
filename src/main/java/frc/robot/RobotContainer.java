// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.operatorConstants;
import frc.robot.commands.CollectVisionData;
import frc.robot.commands.CommandFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Pincer;
import frc.robot.subsystems.Vision;
import lombok.Getter;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 2% deadband //changed from 10%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric preciseAdjustments = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Double> speedChooser = new SendableChooser<Double>();

    public final CommandXboxController driverJoystick = new CommandXboxController(0);
    public final CommandGenericHID keypad = new CommandGenericHID(1);
    public final CommandXboxController operatorJoystick = new CommandXboxController(2);
    public final CommandXboxController testingJoystick = new CommandXboxController(5); // You can put test commands onto here.

    // Subsystem Instances
        @Getter private static final Arm arm = new Arm();
        @Getter private static final Ascender ascender = new Ascender();
        @Getter private static final Pincer pincer = new Pincer();
        @Getter private static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        @Getter private static final Elevator elevator = new Elevator();
        @Getter private static final Lights lights = null;//new Lights();
        @Getter private static final Vision vision = new Vision();
        
        // Command Factory
        public static final CommandFactory commandFactory = new CommandFactory(drivetrain, elevator, arm, pincer, lights);
        public static final CollectVisionData collectVisionData = new CollectVisionData(drivetrain);

        // Custom Triggers
        Trigger LLHasTag = new Trigger(() -> vision.getRobotHasTag());    
        private final BooleanSupplier manualDrivebase = () -> Math.hypot(driverJoystick.getLeftX(), driverJoystick.getLeftY()) > 0.25
                                                                || Math.abs(driverJoystick.getRightX()) > 0.25;

        Trigger closeToReef = new Trigger(()->drivetrain.closeToReef());

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("GoTo_L4", commandFactory.goTolL4());
        NamedCommands.registerCommand("Exhaust_Coral", commandFactory.exhaustCoral());
        NamedCommands.registerCommand("Score_L4", commandFactory.scorelL4(true));
        NamedCommands.registerCommand("Score_L2", commandFactory.scorelL2());
        NamedCommands.registerCommand("Stow_Empty", commandFactory.stow(false, false, true, false));
        NamedCommands.registerCommand("Stow_Coral", commandFactory.stow(true, false, false, true));
        NamedCommands.registerCommand("Reset_To_Vision", Commands.runOnce(() -> drivetrain.resetToVision()));
        NamedCommands.registerCommand("Align_Right", drivetrain.pathPIDToTagRightSelect());
        NamedCommands.registerCommand("Reef_Stall", Commands.run(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.3)), drivetrain));
        NamedCommands.registerCommand("Feed", commandFactory.feedSequential());
        NamedCommands.registerCommand("Pause_For_Feed", new WaitCommand(15).until(() -> pincer.hasCoral()));

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

        speedChooser.setDefaultOption("100% Full Speed", 1.0);
        speedChooser.addOption("75% Speed", 0.75);
        speedChooser.addOption("50% Speed", 0.5);
        speedChooser.addOption("25% Speed", 0.25);

        SmartDashboard.putData("Speed Select", speedChooser);

        configureBindings();

        reefData.reset();

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
                        drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed * speedChooser.getSelected() * elevator.getDrivetrainSpeedMultiplier().getAsDouble()) // Drive
                                                                                                                // forward with
                                                                                                                // negative Y
                                                                                                                // (forward)
                                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed * speedChooser.getSelected() * elevator.getDrivetrainSpeedMultiplier().getAsDouble()) // Drive left with negative X (left)
                                .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate * speedChooser.getSelected() * elevator.getDrivetrainSpeedMultiplier().getAsDouble()) // Drive counterclockwise with
                                .withDeadband(MaxSpeed * 0.1 * elevator.getDrivetrainSpeedMultiplier().getAsDouble())
                                .withRotationalDeadband(MaxAngularRate * 0.1 * elevator.getDrivetrainSpeedMultiplier().getAsDouble())                                                            // negative X (left)
                        ));
                elevator.setDefaultCommand(elevator.holdState());
                arm.setDefaultCommand(arm.holdState());
                pincer.setDefaultCommand(pincer.holdState());
                ascender.setDefaultCommand(ascender.manualClimb(() -> operatorJoystick.getLeftY()));
                vision.setDefaultCommand(Commands.idle(vision));

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
                driverJoystick.start().and(LLHasTag).onTrue(Commands.runOnce(() -> drivetrain.resetToVision()));
                // reset the field-centric heading on back press
                //driverJoystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                driverJoystick.back().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));

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
                operatorJoystick.y().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).negate()).onTrue(commandFactory.scorelL2());
                operatorJoystick.rightBumper().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).negate()).onTrue(commandFactory.scorelL3());
                operatorJoystick.rightTrigger(operatorConstants.triggerBooleanThreshold).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).negate())
                        .and(()-> !arm.facingDownwards())
                                .onTrue(commandFactory.scorelL4(false));
                operatorJoystick.rightTrigger(operatorConstants.triggerBooleanThreshold).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(operatorJoystick.back().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).negate())
                        .and(()-> arm.facingDownwards())
                                .onTrue(commandFactory.scorelL4(true));




                keypad.button(3).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(keypad.button(10).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling)).onTrue(commandFactory.scorelL2());
                keypad.button(2).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(keypad.button(10).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling)).onTrue(commandFactory.scorelL3());
                keypad.button(1).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(keypad.button(10).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling))
                        .and(()-> !arm.facingDownwards())
                                .onTrue(commandFactory.scorelL4(false));
                keypad.button(1).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising).and(keypad.button(10).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling))
                        .and(()-> arm.facingDownwards())
                                .onTrue(commandFactory.scorelL4(true));




                

                



                
                // Feed Commands
                operatorJoystick.pov(0).onTrue(commandFactory.feed());
                keypad.button(5).onTrue(commandFactory.feed());


                keypad.button(13).onTrue(pincer.forceOpen());

                // Stow Commands

                // //empty near top
                // keypad.button(9)
                //         .and(elevator.isNearTop())
                //         .and(() -> !pincer.hasCoral())
                //         .and(() -> !pincer.hasAlgae())
                //                 .onTrue(commandFactory.stow(false, false, true, false));
                // //coral not low
                // keypad.button(9)
                //         .and(() ->pincer.hasCoral())
                //         .and(() -> !elevator.isLow().getAsBoolean())
                //                 .onTrue(commandFactory.stow(true, false, false, false));
                // //all algae
                // keypad.button(9).and(() ->pincer.hasAlgae()).onTrue(commandFactory.stow(false, true, false, false));
                // //low coral
                // keypad.button(9)
                //         .and(elevator.isLow())
                //         .and(() -> pincer.hasCoral())
                //                 .onTrue(commandFactory.stow(true, false, false, true));
                // //not high empty
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
                keypad.button(9)
                        .and(() -> !elevator.isNearTop().getAsBoolean() && !elevator.isLow().getAsBoolean())
                        .and(() -> !pincer.hasCoral())
                        .and(() -> !pincer.hasAlgae())
                                .onTrue(commandFactory.stow(false, false, false, false));

                                keypad.button(9)
                        .and(elevator.isLow())
                        .and(() -> !pincer.hasCoral())
                        .and(() -> !pincer.hasAlgae())
                                .onTrue(commandFactory.stow(false, false, false, true));
                                keypad.button(9)
                                .and(elevator.isNearTop())
                                .and(() -> !pincer.hasCoral())
                                .and(() -> !pincer.hasAlgae())
                                        .onTrue(commandFactory.stow(false, false, true, false));
                        //coral not low
                        keypad.button(9)
                                .and(() ->pincer.hasCoral())
                                .and(() -> !elevator.isLow().getAsBoolean())
                                        .onTrue(commandFactory.stow(true, false, false, false));
                        //all algae
                        keypad.button(9).and(() ->pincer.hasAlgae()).onTrue(commandFactory.stow(false, true, false, false));
                        //low coral
                        keypad.button(9)
                                .and(elevator.isLow())
                                .and(() -> pincer.hasCoral())
                                        .onTrue(commandFactory.stow(true, false, false, true));
                        //not high empty
                        keypad.button(9)
                                .and(() -> !elevator.isNearTop().getAsBoolean() && !elevator.isLow().getAsBoolean())
                                .and(() -> !pincer.hasCoral())
                                .and(() -> !pincer.hasAlgae())
                                        .onTrue(commandFactory.stow(false, false, false, false));
        
                                        keypad.button(9)
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


                keypad.button(10).negate().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).and(keypad.button(3).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising))
                        .onTrue(elevator.setLevelQueue(2));
                keypad.button(10).negate().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).and(keypad.button(2).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising))
                        .onTrue(elevator.setLevelQueue(3));
                keypad.button(10).negate().debounce(operatorConstants.kQueueDebounceTime, DebounceType.kFalling).and(keypad.button(1).debounce(operatorConstants.kQueueDebounceTime, DebounceType.kRising))
                        .onTrue(elevator.setLevelQueue(4));


                //Is High
                keypad.button(6)
                        .and(elevator.isNearTop())
                        .and(() -> !pincer.hasAlgae())
                        .and(()->!LLHasTag.getAsBoolean())
                                .onTrue(commandFactory.stow(true, false, true, false));
                //Is Middle
                keypad.button(6)
                        .and(() -> !elevator.isLow().getAsBoolean())
                        .and(() -> !elevator.isNearTop().getAsBoolean())
                        .and(() -> !pincer.hasAlgae())
                        .and(()->!LLHasTag.getAsBoolean())
                                .onTrue(commandFactory.stow(true, false, false, false));
                //Is Low
                keypad.button(6)
                        .and(elevator.isLow())
                        .and(() -> !pincer.hasAlgae())
                        .and(()->!LLHasTag.getAsBoolean())
                                .onTrue(commandFactory.stow(true, false, false, true));    
        

                        keypad.button(6)
                                .and(elevator.isNearTop())
                                .and(() -> !pincer.hasAlgae())
                                .and(LLHasTag)
                                        .onTrue(elevator.autoReefAlgae().alongWith(arm.coralStow()));
                        //Is Middle
                        keypad.button(6)
                                .and(() -> !elevator.isLow().getAsBoolean())
                                .and(() -> !elevator.isNearTop().getAsBoolean())
                                .and(() -> !pincer.hasAlgae())
                                .and(LLHasTag)
                                        .onTrue(elevator.autoReefAlgae().alongWith(arm.coralStow()));
                        //Is Low
                        keypad.button(6)
                                .and(elevator.isLow())
                                .and(() -> !pincer.hasAlgae())
                                .and(LLHasTag)
                                        .onTrue(elevator.autoReefAlgae().alongWith(arm.coralStow()));          

                //.onTrue(elevator.autoReefAlgae());
                                
                operatorJoystick.start().onTrue(commandFactory.stow(false, true, false, false));
                

                //reef algae

                operatorJoystick.leftBumper().onTrue(
                        commandFactory.reefAlgaeHigh()
                );

                operatorJoystick.leftTrigger(operatorConstants.triggerBooleanThreshold).onTrue(
                        commandFactory.reefAlgaeLow()
                );

                keypad.button(16).onTrue(commandFactory.getLollipop());


                //force score
                
                

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
                //operatorJoystick.b().onTrue(pincer.pincerAlgaeHold());
                operatorJoystick.b().onTrue(pincer.pincerAlgaeHold());
                //operatorJoystick.b().onTrue(pincer.algaeGrab());
                //operatorJoystick.rightStick().onTrue(pincer.pincerFunnel2());
                operatorJoystick.x().onTrue(pincer.pincerFunnel());
                //operatorJoystick.x().onTrue(()->pincer.setPincerOutput(0.278));

                keypad.button(14).onTrue(pincer.pincerFunnel());
                keypad.button(15).onTrue(pincer.pincerAlgaeHold());


        operatorJoystick.pov(270).onTrue(commandFactory.net());
        keypad.button(7).onTrue(commandFactory.net());

        operatorJoystick.pov(180).onTrue(commandFactory.groundAlgae());
        keypad.button(8).onTrue(commandFactory.groundAlgae());

        keypad.button(17).onTrue(commandFactory.exhaustNet()); // Temporary assignment

        //testingJoystick.a().onTrue(commandFactory.net());
        //testingJoystick.b().onTrue(commandFactory.exhaustNet());

        //operatorJoystick.y().toggleOnTrue(pincer.holdIntakeCmd()); */


        // Testing pincer positions
        /*
        testingJoystick.a().onTrue(pincer.pincerFunnel());
        testingJoystick.b().onTrue(pincer.pincerFunnel2());
        testingJoystick.x().onTrue(pincer.algaeScore());
        testingJoystick.y().onTrue(pincer.stowPincer());*/
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}