//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import static frc.robot.Constants.ArmConstants.*;

import java.util.ArrayList;
import java.util.List;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    private final SparkClosedLoopController pivotController;
    private final AbsoluteEncoder pivotAbsEncoder;

    private boolean pivotPIDUpdated = false;

    // Feedforward controller for arm motion (helps to predict required motor
    // output)
    private final ArmFeedforward pivotFeedforward;
    private double pivotSetpoint, FF;

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
        pivotConfig.encoder.positionConversionFactor(kPositionConversionFactor);
        pivotConfig.closedLoop
                .feedbackSensor(kPivotMotorFeedbackSensor)
                .pid(kPivotP, kPivotI, kPivotD)
                .outputRange(kPivotMotorMinOutput, kPivotMotorMaxOutput);
        pivotConfig.absoluteEncoder
                .zeroOffset(kPivotMotorAbsoluteEncoderOffset);
        pivotConfig.softLimit
        pivotConfig
            .absoluteEncoder
                .zeroOffset(kPivotMotorAbsoluteEncoderOffset)
                .zeroCentered(kPivotMotorAbsoluteEncoderZeroCentered);
        pivotConfig
            .softLimit
                .reverseSoftLimitEnabled(kSoftLimitsEnabled)
                .forwardSoftLimitEnabled(kSoftLimitsEnabled)
                .forwardSoftLimit(kPivotMaxAngle)
                .reverseSoftLimit(kPivotMinAngle);

        // Initialize Feedforward
        pivotSetpoint = kStowPosition;
        FF = 0;
        goalState = new TrapezoidProfile.State();
        startState = new TrapezoidProfile.State();
        currentState = new TrapezoidProfile.State();

        pivotFeedforward = new ArmFeedforward(kPivotMotorks, kPivotMotorkg, kPivotMotorkv, kPivotMotorka);

        pivotProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kPivotMotorMaxVelocity, kPivotMotorMaxAcceleration));

        timer.start();

        // Burn motor configurations
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Constants.kMotorBurnDelay();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        // Absolute Encoder
        builder.addDoubleProperty("Pivot Absolute Position", () -> pivotAbsEncoder.getPosition(), null);
        builder.addDoubleProperty("Pivot Absolute Velocity", () -> pivotAbsEncoder.getVelocity(), null);
        // Pivot Motor
        builder.addDoubleProperty("Pivot Motor Output", () -> pivotMotor.getAppliedOutput(), null);
        builder.addDoubleProperty("Pivot Motor Temperature", () -> pivotMotor.getMotorTemperature(), null);
        builder.addDoubleProperty("Pivot Motor Output Current", () -> pivotMotor.getOutputCurrent(), null);

        // Motion Profile
        builder.addDoubleProperty("Pivot Setpoint", () -> pivotSetpoint, null);
        builder.addDoubleProperty("Pivot Profile Timer", () -> timer.get(), null);
        builder.addDoubleProperty("Pivot Profile Current Position", () -> currentState.position, null);
        builder.addDoubleProperty("Pivot Profile Current Velocity", () -> currentState.velocity, null);
        builder.addDoubleProperty("Pivot Profile Position Error Deg",
                () -> currentState.position - pivotAbsEncoder.getPosition(), null);
        builder.addDoubleProperty("Pivot Profile Velocity Error Deg",
                () -> currentState.velocity - pivotAbsEncoder.getVelocity(), null);
        builder.addDoubleProperty("Pivot Profile Position Error Deg", () -> (currentState.position - pivotAbsEncoder.getPosition()) * 360, null);
        builder.addDoubleProperty("Pivot Profile Velocity Error Deg", () -> currentState.velocity - pivotAbsEncoder.getVelocity(), null);
        builder.addDoubleProperty("Pivot Profile Goal Position", () -> goalState.position, null);
        builder.addDoubleProperty("Pivot Profile Goal Velocity", () -> goalState.velocity, null);

        // PID Tuning
        builder.addDoubleProperty("Pivot kP", () -> kPivotP, value -> {
            kPivotP = value;
            pivotPIDUpdated = true;
        });
        builder.addDoubleProperty("Pivot kI", () -> kPivotI, value -> {
            kPivotI = value;
            pivotPIDUpdated = true;
        });
        builder.addDoubleProperty("Pivot kD", () -> kPivotD, value -> {
            kPivotD = value;
            pivotPIDUpdated = true;
        });

        builder.addDoubleProperty("Pivot kP", () -> kPivotP, value -> { kPivotP = value; pivotPIDUpdated = true;});
        builder.addDoubleProperty("Pivot kI", () -> kPivotI, value -> { kPivotI = value; pivotPIDUpdated = true;});
        builder.addDoubleProperty("Pivot kD", () -> kPivotD, value -> { kPivotD = value; pivotPIDUpdated = true;});
        
        builder.addDoubleProperty("Pivot Feedforward Output", () -> FF, null);
    }

    @Override
    public void periodic() {
        if (pivotPIDUpdated) {
            pivotConfig.closedLoop.pid(kPivotP, kPivotI, kPivotD);
            pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            pivotPIDUpdated = false;
        }

        SmartDashboard.putData(this);
        List<Integer> keyDown = new ArrayList<>();
        keyDown = keypad.keys;
        keypad.keyMode mode = keypad.mode;
        // Constants.reef.reefLs L = Constants.reef.reefLs.STOW;
        // for(int i : keyDown){

        // if(Constants.reef.lMap.containsKey(i)){
        // L = Constants.reef.lMap.get(i);
        // }
        // }

        // if(mode== keypad.keyMode.SCORE){
        // if(L == Constants.reef.reefLs.lL1 || L == Constants.reef.reefLs.rL1){
        // this.pivotToL1();
        // }
        // if(L == Constants.reef.reefLs.lL2 || L == Constants.reef.reefLs.rL2){
        // this.pivotToL2L3();
        // }
        // if(L == Constants.reef.reefLs.lL3 || L == Constants.reef.reefLs.rL3){
        // this.pivotToL2L3();
        // }
        // if(L == Constants.reef.reefLs.lL4 || L == Constants.reef.reefLs.rL4){
        // this.pivotToL4();
        // }
        // }

    }

    /**
     * Follows a motion profile to rotate the arm to the new setpoint
     * 
     * @param newSetpoint Setpoint for the arm pivot to go to
     * @return Command
     */
    private Command pivotToSetpoint(double newSetpoint) {
        return startRun(
            () -> {
                this.pivotSetpoint = Constants.kClamp(newSetpoint, kPivotMinAngle, kPivotMaxAngle);
                timer.reset();
                startState = new TrapezoidProfile.State(pivotAbsEncoder.getPosition(), pivotAbsEncoder.getVelocity());
                goalState = new TrapezoidProfile.State(this.pivotSetpoint, 0);
            },
            () -> {
                currentState = pivotProfile.calculate(timer.get(), startState, goalState);
                setPivotOutput(currentState.position, currentState.velocity); // rotations
            })
            .until(() -> pivotProfile.isFinished(timer.get()))
            .withName("Go To Reference");
                () -> {
                    this.pivotSetpoint = Constants.kClamp(newSetpoint, kPivotMinAngle, kPivotMaxAngle);
                    timer.reset();
                    startState = new TrapezoidProfile.State(pivotAbsEncoder.getPosition(),
                            pivotAbsEncoder.getVelocity());
                    goalState = new TrapezoidProfile.State(this.pivotSetpoint, 0);
                },
                () -> {
                    currentState = pivotProfile.calculate(timer.get(), startState, goalState);
                    setPivotOutput(currentState.position, currentState.velocity);
                })
                .until(() -> pivotProfile.isFinished(timer.get()))
                .withName("Go To Reference");
    }

    /**
     * Sets the pivot to the position for stowing
     * 
     * @return Commmand
     */
    public Command stowPivot() {
        return pivotToSetpoint(kStowPosition);
    }

    /**
     * Sets the pivot to the position for intaking from the feeding station
     * 
     * @return Commmand
     */
    public Command pivotToFeed() {
        return pivotToSetpoint(kFeedPosition);
    }

    /**
     * Sets the pivot to the position for scoring on L1
     * 
     * @return Commmand
     */
    public Command pivotToL1() {
        return pivotToSetpoint(kL1Position);
    }

    public Command moveToPoint(double point) {
        return pivotToSetpoint(point);
    }

    /**
     * Sets the pivot to the position for scoring on L2 & L3
     * 
     * @return Commmand
     */
    public Command pivotToL2L3() {
        return pivotToSetpoint(kL2L3Position);
    }

    /**
     * Sets the pivot to the position for scoring on L4
     * 
     * @return Commmand
     */
    public Command pivotToL4() {
        return pivotToSetpoint(kL4Position);
    }

    /**
     * Holds the arm pivot at the current angle setpoint
     * 
     * @return Command
     */
    public Command holdState() {
        return run(() -> {
            setPivotOutput(pivotSetpoint, 0);
        }).withName("Hold State");
    }

    public Command changeSetpointInstant(double newSetpoint) {
        return runOnce(() -> pivotSetpoint = newSetpoint);
    }

    public Command simpleSetMotorOutput(DoubleSupplier input) {
        return run(() -> {
            pivotMotor.set(input.getAsDouble());
        });
    }

    public Command algaeStow() {
        return pivotToSetpoint(Constants.ArmConstants.setPoints.get(
                Constants.robotStates.pivotElevatorStates.ALGAESTOW));
    }

    public Command coralStow() {
        return pivotToSetpoint(Constants.ArmConstants.setPoints.get(
                Constants.robotStates.pivotElevatorStates.CORALSTOW));
    }

    public Command emptyStow() {
        return pivotToSetpoint(Constants.ArmConstants.setPoints.get(
                Constants.robotStates.pivotElevatorStates.EMPTYSTOW));
    }

    public Command goToNet() {
        return pivotToSetpoint(Constants.ArmConstants.setPoints.get(
            Constants.robotStates.pivotElevatorStates.NET
        ));
    }

    public Command reefAlgaeHigh(){
        return pivotToSetpoint(Constants.ArmConstants.setPoints.get(
            Constants.robotStates.pivotElevatorStates.REEFALGAEHIGH
        ));
    }
    public Command reefAlgaeLow(){
        return pivotToSetpoint(Constants.ArmConstants.setPoints.get(
            Constants.robotStates.pivotElevatorStates.REEFALGAELOW
        ));
    }
    public Command goToProcessor(){
        return pivotToSetpoint(Constants.ArmConstants.setPoints.get(
          Constants.robotStates.pivotElevatorStates.PROCESSOR
        ));
      }

    /**
     * Sets the PID setpoint with the calculated feedforward to the pivot motor
     * 
     * @param position position setpoint (feedforward & PID)
     * @param velocity velocity setpoint (feedforward)
     */
    private void setPivotOutput(double position, double velocity) {
        FF = pivotFeedforward.calculate(position * 2.0 * Math.PI, velocity);
        pivotController.setReference(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, FF,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

}
