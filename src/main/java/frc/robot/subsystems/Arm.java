//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import static frc.robot.Constants.ArmConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/*
 * Joseph's changes notes:
 * - removed absEncoder, will be plugged strait into the spark max
 * - removed armPID, will use spark max internal controller
 * - removed simulation override, not going to use that this season. would be a cool offseason project
 * - made the arm constants import static so I don't have to type ArmConstants. ...
 * - added a constant function for motor burn delay
 */

public class Arm extends SubsystemBase {

    private final SparkMax armPivotMotor = new SparkMax(kArmPivotMotorId, MotorType.kBrushless);
    private final SparkMaxConfig armPivotConfig  = new SparkMaxConfig();
    private final SparkClosedLoopController armPivotController;
    private final AbsoluteEncoder armPivotAbsEncoder;

    private final SparkMax armPincerMotor = new SparkMax(kArmPincerMotorId, MotorType.kBrushless);
    private final SparkMaxConfig armPincerConfig  = new SparkMaxConfig();
    private final SparkClosedLoopController armPincerController;

    private final SparkMax armIntakeMotor = new SparkMax(kArmIntakeMotorId, MotorType.kBrushless);
    private final SparkMaxConfig armIntakeConfig  = new SparkMaxConfig();

    // Feedforward controller for arm motion (helps to predict required motor output)
    private final ArmFeedforward armPivotFeedforward;
    private double armPivotReference;
    /* not using a feedforward controller for the pincer since gravity's effect will change due to the pivot angle
    * and the mechanism will be relatively light compared to the torque of the motor.
    */

    // Motion profiling and state tracking
    private final TrapezoidProfile armProfile; // Profile for trapezoidal motion
    private TrapezoidProfile.State armGoalState, armStartState, armCurrentState; // States used for motion control
    private final Timer armTimer = new Timer();

    // Photogate (beam break)
    private final DigitalInput armPhotogate = new DigitalInput(kArmPhotogateId);

    // Potentially also 2 limit switches - DigitalInput class
    // There will also be a USB camera that I don't think will be represented here



    public Arm() {
        // Grab objects from spark maxs
        armPivotAbsEncoder = armPivotMotor.getAbsoluteEncoder();
        armPivotController = armPincerMotor.getClosedLoopController();
        armPincerController = armPincerMotor.getClosedLoopController();

        // Build motor configs
        armPivotConfig
            .inverted(kArmPivotMotorInverted)
            .idleMode(kArmPivotMotorIdleMode)
            .smartCurrentLimit(kArmPivotMotorSmartCurrentLimit);
        armPivotConfig
            .closedLoop
                .feedbackSensor(kArmPivotMotorFeedbackSensor)
                .pid(kArmPivotP, kArmPivotI, kArmPivotD)
                .outputRange(kArmPivotMotorMinOutput, kArmPivotMotorMaxOutput);
        armPivotConfig
            .absoluteEncoder
                .zeroOffset(kArmPivotMotorAbsoluteEncoderOffset);

        armPincerConfig
            .inverted(kArmPincerMotorInverted)
            .idleMode(kArmPincerMotorIdleMode)
            .smartCurrentLimit(kArmPincerMotorSmartCurrentLimit);
        armPincerConfig
            .closedLoop
                .feedbackSensor(kArmPincerMotorFeedbackSensor)
                .pid(kArmPincerP, kArmPincerI, kArmPincerD)
                .outputRange(kArmPincerMotorMinOutput, kArmPincerMotorMaxOutput);
        armPincerConfig
            .absoluteEncoder
                .zeroOffset(kArmPincerMotorAbsoluteEncoderOffset);

        armIntakeConfig
            .inverted(kArmIntakeMotorInverted)
            .idleMode(kArmIntakeMotorIdleMode)
            .smartCurrentLimit(kArmAbsEncoderId);
        armIntakeConfig
            .closedLoop
                .pid(kArmIntakeP, kArmIntakeI, kArmIntakeD)
                .outputRange(kArmIntakeMotorMinOutput, kArmIntakeMotorMaxOutput);
        

        // Initialize Feedforward 
        armPivotReference = 0.0;
        armGoalState = new TrapezoidProfile.State(); 
        armStartState = new TrapezoidProfile.State(); 
        armCurrentState = new TrapezoidProfile.State(); 

        armPivotFeedforward = new ArmFeedforward(kArmPivotMotorks, kArmPivotMotorkg, kArmPivotMotorkv, kArmPivotMotorka);
        
        armProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kArmPivotMotorMaxVelocity, kArmPivotMotorMaxAcceleration));

        armTimer.start();


        // Burn motor configurations
        armPivotMotor.configure(armPincerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Constants.kMotorBurnDelay();
        armPincerMotor.configure(armPincerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Constants.kMotorBurnDelay();
        armIntakeMotor.configure(armIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Constants.kMotorBurnDelay();

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder); // Initialize the sendable builder

    }

    @Override
    public void periodic() {

    }

    public Command pivotToReference(double reference){
        return startRun(
            () -> {
                this.armPivotReference = Constants.kClamp(reference, kArmPivotMinAngle, kArmPivotMaxAngle);
                armTimer.reset();
                armStartState = new TrapezoidProfile.State(armPivotAbsEncoder.getPosition(), armPivotAbsEncoder.getVelocity());
                armGoalState = new TrapezoidProfile.State(this.armPivotReference, 0);
            },
            ()->{
                armCurrentState = armProfile.calculate(armTimer.get(), armStartState, armGoalState);
                setPivotOutput(armCurrentState.position, armCurrentState.velocity);
            })
            .until(null)
            .withName("Go To Reference");
    }

    public Command holdState(){
        return run(()->{
            setPivotOutput(armPivotReference, 0);
        });
    }

    public Command intake(){
        return null;
    }

    public Command exhaust(){
        return null;
    }

    public Command stopRollers(){
        return null;
    }

    private void setPivotOutput(double position, double velocity){
        double FF = armPivotFeedforward.calculate(position, velocity);
        armPivotController.setReference(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, FF, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }
    

}
