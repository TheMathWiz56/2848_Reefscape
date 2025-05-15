//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.reefData;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.PincerConstants.kStowPosition;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class Elevator extends SubsystemBase {

  private final TalonFX elevatorMotor = new TalonFX(kMotorId);

  // Motor configs
  TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();

  private final DigitalInput elevatorLimitSwitchBottom = new DigitalInput(
      kLimitSwitchBottomId);

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kFeedforwardKs,
      kFeedforwardKg, kFeedforwardKv, kFeedforwardKa);

  // Trapezoid profile for feedforward
  private final TrapezoidProfile elevatorTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      kMaxVelocity, kMaxAcceleration));

  // Trapezoid profile states
  private TrapezoidProfile.State startState = new TrapezoidProfile.State();
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State();

  private boolean keyHeld = false;

  // Timer for trapezoid profile
  private final Timer timer = new Timer();

  private double currentSetpoint = 0.0;

  private boolean isZeroed = false;

  private IntSupplier levelQueue = () -> 4;

  public Elevator() {

    // Set motor configurations
    elevatorMotorConfig.MotorOutput
        .withInverted(kMotorInverted)
        .withNeutralMode(kMotorIdleMode);

    elevatorMotorConfig.CurrentLimits
        .withSupplyCurrentLimit(kStallCurrent);

    // Missing feedback sensor, izone, imaxaccum, outputrange
    elevatorMotorConfig.Slot0
        .withKP(kP)
        .withKI(kI)
        .withKD(kD);

    // Missing encoder position conversion factor, velocity conversion factor, zero
    // offset

    // Timer start
    timer.start();

    // Apply the motor configurations, set PID to slot 0
    elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
  }

  public void setMotorVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);

  }

  /**
     * Changes the setpoint of the elevator, beginning to move it to the position. 
     * @param setpoint The new setpoint the robot should move to.
     */
  public void setElevatorSetpoint(double setpoint) {
    currentSetpoint = setpoint;
    timer.reset();
    startState = new TrapezoidProfile.State(elevatorMotor.getPosition().getValueAsDouble(),
        elevatorMotor.getVelocity().getValueAsDouble());
    goalState = new TrapezoidProfile.State(setpoint, 0.0);
  }

  // Set pivot output based on position, velocity
  public void setMotorOutput(double position, double velocity) {
    // Units might be messed up
    PositionVoltage request = new PositionVoltage(position).withSlot(0)
        .withFeedForward(feedforward.calculate(velocity, position));

    SmartDashboard.putNumber("Feedforward Output", feedforward.calculate(velocity, position));

    elevatorMotor.setControl(request);
  }

  // Default command - hold position
  public Command holdState() {
    if (!isZeroed)
      return autoZeroEncoder();
    else
      return this.run(() -> {
        setMotorOutput(currentSetpoint, 0.0);
      }).withName("Elevator Default Command");
  }

  // Command to go to position
  public Command goToPosition(double position, String positionName) {
    return this.startRun(() -> {
      setElevatorSetpoint(position);
    }, () -> {
      currentState = elevatorTrapezoidProfile.calculate(timer.get(), startState, goalState);
      setMotorOutput(currentState.position, currentState.velocity);
    }).until(() -> elevatorTrapezoidProfile.isFinished(timer.get()))
        .withName("Go to " + positionName);
  }

  public Command goToPosition(double position) {
    return goToPosition(position, "Position");
  }

  public Command goToPosition() {
    return goToPosition(currentSetpoint, "Position").withName("AutoReprofile");
  }

  public Command goUpByDistance(double distance) {
    return goToPosition(elevatorMotor.getPosition().getValueAsDouble() - distance, "Up By " + distance);
  }

  // Reminder: negative elevator value = up

  // For going up, use with arm.facingUpwards()
  public Command goToPositionClampUpward(double position, BooleanSupplier armFacingUpwards, String positionName) {
    if (armFacingUpwards.getAsBoolean())
      return goToPosition(MathUtil.clamp(position, kUpwardsSafePosition, 1000), positionName + " (Upwards Clamp)");
    else
      return goToPosition(position, positionName);
  }
  
  // For going up, use with arm.facingUpwards()
  public Command goToPositionClampUpward(double position, BooleanSupplier armFacingUpwards) {
    return goToPositionClampUpward(position, armFacingUpwards, "Position");
  }

  // For going down, use with arm.facingDownwards()
  public Command goToPositionClampDownward(double position, BooleanSupplier armFacingDownwards, String positionName) {
    if (armFacingDownwards.getAsBoolean())
      return goToPosition(MathUtil.clamp(position, -1000, kDownwardsSafePosition), positionName + " (Downwards Clamp)");
    else
      return goToPosition(position, positionName);
  }
  
  // For going down, use with arm.facingDownwards()
  public Command goToPositionClampDownward(double position, BooleanSupplier armFacingDownwards) {
    return goToPositionClampDownward(position, armFacingDownwards, "Position");
  }



  // Commands to go to various pre-defined positions

  // public Command goToL1(int reef,int L) {
  // return this.startEnd(()->goToPosition(kSetpointL1, "L1"),()->
  // reefData.update(reef,L,false));
  // }

  public Command goToL1() {
    return goToPosition(kSetpointL1, "L1");
  }

  public Command goToL2() {
    return goToPosition(kSetpointL2, "L2");
  }

  public Command goToL3() {
    return goToPosition(kSetpointL3, "L3");
  }

  public Command goToL4() {
    return goToPosition(kSetpointL4, "L4");
  }

  public Command goToL4(BooleanSupplier goingUpwards) {
    return goToPositionClampUpward(kSetpointL4, goingUpwards, "L4");
  }

  public Command goToFeed() {
    return goToPosition(kSetpointFeed, "Feed");
  }

  public Command algaeStow() {
    return goToPosition(Constants.ElevatorConstants.setPoints.get(
        Constants.robotStates.pivotElevatorStates.ALGAESTOW));
  }

  public Command coralStow() {
    // return goToPosition(Constants.ElevatorConstants.setPoints.get(
    // Constants.robotStates.pivotElevatorStates.CORALSTOW
    return goToPosition(kSetpointStowCoral, "Stow Coral");
  }

  public Command coralStow(BooleanSupplier goingDownwards) {
    return goToPositionClampDownward(kSetpointStowCoral, goingDownwards, "Stow Coral");
  }

  public Command emptyStow() {
    return goToPosition(Constants.ElevatorConstants.setPoints.get(
        Constants.robotStates.pivotElevatorStates.EMPTYSTOW));
  }

  public Command goToNet() {
    return goToPosition(Constants.ElevatorConstants.setPoints.get(
        Constants.robotStates.pivotElevatorStates.NET));
  }

  public Command goToStow() {
    return goToPosition(kSetpointStow, "Stow");
  }

  public Command goToStow(BooleanSupplier goingDownwards) {
    return goToPositionClampDownward(kSetpointStow, goingDownwards, "Stow");
  }

  public Command goToProcessor() {
    return goToPosition(Constants.ElevatorConstants.setPoints.get(
        Constants.robotStates.pivotElevatorStates.PROCESSOR));
  }

  public Command reefAlgaeHigh() {
    return goToPosition(
        Constants.ElevatorConstants.setPoints.get(
            Constants.robotStates.pivotElevatorStates.REEFALGAEHIGH));
  }

  public Command reefAlgaeLow() {
    return goToPosition(
        Constants.ElevatorConstants.setPoints.get(
            Constants.robotStates.pivotElevatorStates.REEFALGAELOW));
  }

  public Command reefAlgaeHighAuto() {
    return goToPosition(Constants.ElevatorConstants.kSetpointReefAlgaeHighAuto);
  }

  public Command reefAlgaeLowAuto() {
    return goToPosition(Constants.ElevatorConstants.kSetpointReefAlgaeLowAuto);
  }




  public Command goToGroundAlgae(){
    return goToPosition(
      Constants.ElevatorConstants.setPoints.get(
        Constants.robotStates.pivotElevatorStates.GROUNDALGAE
      )
    );
  }
  public Command goToL(Constants.reef.reefLs L){
    return this.startRun(() -> {
      setElevatorSetpoint(Constants.ElevatorConstants.setPoints.get(Constants.reef.reefToState.get(L)));
      //setElevatorSetpoint(-26);
    }, () -> {
      currentState = elevatorTrapezoidProfile.calculate(timer.get(), startState, goalState);
      setMotorOutput(currentState.position, currentState.velocity);
    }).until(() -> elevatorTrapezoidProfile.isFinished(timer.get()))
        .withName("Go to " + L.name());
    //return this.startEnd(()->goToPosition(kSetpointL1, "L1"),()-> reefData.update(reef,L,false));
  }


  public void zeroEncoder() {
    elevatorMotor.setPosition(0.0);
    isZeroed = true;
  }

  public double LtoSetPoint(Constants.reef.reefLs L) {
    if (L.equals(Constants.reef.reefLs.lL1) || L.equals(Constants.reef.reefLs.rL1)) {
      return kSetpointL1;
    }
    if (L.equals(Constants.reef.reefLs.lL2) || L.equals(Constants.reef.reefLs.rL2)) {
      return kSetpointL2;
    }
    if (L.equals(Constants.reef.reefLs.lL3) || L.equals(Constants.reef.reefLs.rL3)) {
      return kSetpointL3;
    }
    if (L.equals(Constants.reef.reefLs.lL4) || L.equals(Constants.reef.reefLs.rL4)) {
      return kSetpointL4;
    }
    return 0;
  }

  public Command goToL(Constants.reef.reefLs L, int reef) {
    return this.startRun(() -> {
      setElevatorSetpoint(Constants.ElevatorConstants.setPoints.get(Constants.reef.reefToState.get(L)));
    }, () -> {
      currentState = elevatorTrapezoidProfile.calculate(timer.get(), startState, goalState);
      setMotorOutput(currentState.position, currentState.velocity);
    }).until(() -> elevatorTrapezoidProfile.isFinished(timer.get()))
        .withName("Go to " + L.name());
    // return this.startEnd(()->goToPosition(kSetpointL1, "L1"),()->
    // reefData.update(reef,L,false));
  }

  public Command autoZeroEncoder() {
    return run(() -> setMotorVoltage(1.5))
        .until(() -> elevatorMotor.getSupplyCurrent().getValueAsDouble() > kZeroingCurrent || !elevatorLimitSwitchBottom.get())
        .andThen(runOnce(() -> {
          zeroEncoder();
          setMotorVoltage(0.0);
          this.setDefaultCommand(holdState());
        })).withName("Zero Encoder");
  }

  @Override
  public void periodic() {
    if(!isZeroed){
      if(!elevatorLimitSwitchBottom.get()){
        this.setDefaultCommand(holdState());
        isZeroed = true;
      }
    }

    if (autoReprofile()){
      CommandScheduler.getInstance().schedule(this.goToPosition(currentSetpoint, "AutoReProfile"));
    }
    
    SmartDashboard.putData(this);
    }

  @Override
  public void simulationPeriodic() {

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder); // Not sure why we need this

    // Motor information
    builder.addDoubleProperty("Elevator Motor Temperature", () -> elevatorMotor.getDeviceTemp().getValueAsDouble(),
        null);
    builder.addDoubleProperty("Elevator Motor Closed Loop Output",
        () -> elevatorMotor.getClosedLoopOutput().getValueAsDouble(), null);
    builder.addDoubleProperty("Elevator Motor Output Current",
        () -> elevatorMotor.getSupplyCurrent().getValueAsDouble(), null);

    builder.addDoubleProperty("Torque Current (Amps)",
        () -> Math.abs(elevatorMotor.getTorqueCurrent().getValueAsDouble()),
        null);
    builder.addDoubleProperty("Supply Current (Amps)",
        () -> Math.abs(elevatorMotor.getSupplyCurrent().getValueAsDouble()),
        null);
    builder.addDoubleProperty("Encoder Output (Rotations)", () -> elevatorMotor.getPosition().getValueAsDouble(),
        (input) -> elevatorMotor.setPosition(input));

    builder.addDoubleProperty("Setpoint", () -> currentSetpoint, null);
    builder.addDoubleProperty("Profile Current Setpoint", () -> currentState.position, null);

    builder.addDoubleProperty("Velocity", () -> elevatorMotor.getVelocity().getValueAsDouble(), null);

    builder.addDoubleProperty("Output", () -> elevatorMotor.get(), null);

    builder.addDoubleProperty("Profile Current Velocity", () -> currentState.velocity, null);

    builder.addDoubleProperty("Timer", () -> timer.get(), null);

    builder.addBooleanProperty("Limit Switch State", () -> elevatorLimitSwitchBottom.get(), null);
    builder.addBooleanProperty("Is Zeroed", () -> isZeroed, null);

    builder.addDoubleProperty("Drivetrain Speed Multiplier", getDrivetrainSpeedMultiplier(), null);

    builder.addDoubleProperty("Velocity Error", () -> elevatorMotor.getVelocity().getValueAsDouble() - currentState.velocity, null);

    builder.addDoubleProperty("LevelQueue", () -> levelQueue.getAsInt(), value -> {levelQueue = () -> (int)value;});

    builder.addBooleanProperty("L2 Queued", () -> levelQueue.getAsInt() == 2, null);
    builder.addBooleanProperty("L3 Queued", () -> levelQueue.getAsInt() == 3, null);
    builder.addBooleanProperty("L4 Queued", () -> levelQueue.getAsInt() == 4, null);

    builder.addBooleanProperty("Is Holding State", () -> isHoldingState(), null);
    builder.addBooleanProperty("Autoreprofile", () -> autoReprofile(), null);
  }

  public boolean isHoldingState(){
    Command currentCommand = this.getCurrentCommand();
    if (currentCommand != null){
      return currentCommand.getName() == holdState().getName();
    }
    return false;
  }

  public boolean autoReprofile (){
    return isHoldingState() && Math.abs(elevatorMotor.getPosition().getValueAsDouble() - currentSetpoint) > autoReProfileThreshold;
  }

  public BooleanSupplier isHigh() {
    return () -> elevatorMotor.getPosition().getValueAsDouble() < -20;
  }

  public BooleanSupplier isNearTop() {
    return () -> elevatorMotor.getPosition().getValueAsDouble() < -35;
  }

  public BooleanSupplier isLow(){
    return () -> elevatorMotor.getPosition().getValueAsDouble() > -12.5;
  }

  /**
   * 0 - Low
   * 1 - Middle
   * 2 - High
   * @return The Elevator State
   */
  public int getState(){
    if (isLow().getAsBoolean()){
      return 0;
    }
    if (!isLow().getAsBoolean() && !isHigh().getAsBoolean()){
      return 1;
    }
    return 2;
  }

  /**
     * Returns a supplier that is used to slow down the elevator when it is extended. Reads the elevator height itself.
     * 
     * @return A {@code DoubleSupplier} that returns a multiplier for the speed based upon elevator height.
     */
  public DoubleSupplier getDrivetrainSpeedMultiplier() {
    return () -> MathUtil.clamp((0.034 * elevatorMotor.getPosition().getValueAsDouble()) + 1.51, 0.15, 1); //old: 0.043x + 1.87
  }

  public Command setLevelQueue(int level){
    return runOnce(() -> levelQueue = () -> level);
  }

  public int getLevelQueue(){
    return levelQueue.getAsInt();
  }

}