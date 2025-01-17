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
 * - split pincer and intake into a seperate subsystem
 * 
 * Notes:
 * Depending on the used sensors, we may not know where the elevator/arm is on startup. Need to have some checks to restrict control until 0'd
 */

 public class Arm extends SubsystemBase {

    private final SparkMax pivotMotor = new SparkMax(kPivotMotorId, MotorType.kBrushless);
    private final SparkMaxConfig pivotConfig  = new SparkMaxConfig();
    private final SparkClosedLoopController pivotController;
    private final AbsoluteEncoder pivotAbsEncoder;

    // Feedforward controller for arm motion (helps to predict required motor output)
    private final ArmFeedforward pivotFeedforward;
    private double pivotReference;

    // Motion profiling and state tracking
    private final TrapezoidProfile pivotProfile; // Profile for trapezoidal motion
    private TrapezoidProfile.State goalState, startState, currentState; // States used for motion control
    private final Timer timer = new Timer();

    public Arm() {
        // Grab objects from spark maxs
        pivotAbsEncoder = pivotMotor.getAbsoluteEncoder();
        pivotController = pivotMotor.getClosedLoopController();

        // Build motor configs
        pivotConfig
            .inverted(kPivotMotorInverted)
            .idleMode(kPivotMotorIdleMode)
            .smartCurrentLimit(kPivotMotorSmartCurrentLimit);
        pivotConfig
            .encoder.positionConversionFactor(kPositionConversionFactor);
        pivotConfig
            .closedLoop
                .feedbackSensor(kPivotMotorFeedbackSensor)
                .pid(kPivotP, kPivotI, kPivotD)
                .outputRange(kPivotMotorMinOutput, kPivotMotorMaxOutput);
        pivotConfig
            .absoluteEncoder
                .zeroOffset(kPivotMotorAbsoluteEncoderOffset);
        pivotConfig
            .softLimit
                .reverseSoftLimitEnabled(kSoftLimitsEnabled)
                .forwardSoftLimitEnabled(kSoftLimitsEnabled)
                .forwardSoftLimit(kPivotMaxAngle)
                .reverseSoftLimit(kPivotMinAngle);        

        // Initialize Feedforward 
        pivotReference = kStowPosition;
        goalState = new TrapezoidProfile.State(); 
        startState = new TrapezoidProfile.State(); 
        currentState = new TrapezoidProfile.State(); 

        pivotFeedforward = new ArmFeedforward(kPivotMotorks, kPivotMotorkg, kPivotMotorkv, kPivotMotorka);
        
        pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kPivotMotorMaxVelocity, kPivotMotorMaxAcceleration));

        timer.start();

        // Burn motor configurations
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Constants.kMotorBurnDelay();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder); // Initialize the sendable builder
    }

    @Override
    public void periodic() {
    }

    /**
     * Follows a motion profile to rotate the arm to the new setpoint
     * 
     * @param reference Setpoint for the arm pivot to go to
     * @return Command
     */
    public Command pivotToReference(double reference) {
        return startRun(
            () -> {
                this.pivotReference = Constants.kClamp(reference, kPivotMinAngle, kPivotMaxAngle);
                timer.reset();
                startState = new TrapezoidProfile.State(pivotAbsEncoder.getPosition(), pivotAbsEncoder.getVelocity());
                goalState = new TrapezoidProfile.State(this.pivotReference, 0);
            },
            () -> {
                currentState = pivotProfile.calculate(timer.get(), startState, goalState);
                setPivotOutput(currentState.position, currentState.velocity);
            })
            .until(() -> pivotProfile.isFinished(timer.get()))
            .withName("Go To Reference");
    }

    /**
     * Holds the arm pivot at the current angle setpoint
     * 
     * @return Command
     */
    public Command holdState() {
        return run(() -> {
            setPivotOutput(pivotReference, 0);
        });
    }

    /**
     * Sets the PID setpoint with the calculated feedforward to the pivot motor
     * 
     * @param position position setpoint (feedforward / PID)
     * @param velocity velocity setpoint (feedforward)
     */
    private void setPivotOutput(double position, double velocity) {
        double FF = pivotFeedforward.calculate(position, velocity);
        pivotController.setReference(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, FF, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }
    
}

