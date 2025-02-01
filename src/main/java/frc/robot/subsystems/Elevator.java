//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

/*
 * Joseph Notes
 * - [Done] one of the elevator motors should be configured to follow the other, not just setting them both to the same output
 * - [Half finished] If we decide to use the internal encoder or an absolute encoder to measure the elevator's position we can use the internal
 * pid controller on the elevator. 
 * - [Done for the LaserCan setup] Should also have a feedforward controller for the elevator, results in smoothing motion since it allows you to create a motion
 * profile. Look at the arm branch for some ideas of how to imlpement feedforward, I can also help.
 * - [Added 2 commands] You can either use methods like public void setMotors ()... or you can use public Command setMotors() with command
 * factories to simplify code and remove boiler plate code
 * - [Done for LaserCan] Add a sendable builder and put all sensor information, motor outputs, setpoints, setpoints errors, PID outputs, Feedforward outputs etc.
 * in periodic send the sendable object to the dashboard for debugging and logging
 */

public class Elevator extends SubsystemBase {

  private final TalonFX elevatorMotor = new TalonFX(kMotorId);

  // Motor configs
  TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();

  // Limit switches
  /*
   * [Done] Should be bound to a trigger in robotcontainer that calls something
   * like
   * public Command atHardLimit()
   * Which 0's the motor output if it is trying to drive the elevator into an
   * unsafe state or allows the elevator to move
   * if the motors are trying to move it to a safe state.
   * ie. we are at the Bottom limit switch and trying to drive the elevator down
   * more, don't allow motor output
   * if we are at the bottom limit switch and tring to drive the elevator up,
   * allow motor output
   */
  private final DigitalInput elevatorLimitSwitchTop = new DigitalInput(kLimitSwitchTopId);
  private final DigitalInput elevatorLimitSwitchBottom = new DigitalInput(
      kLimitSwitchBottomId);

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kFeedforwardKs,
      kFeedforwardKg, kFeedforwardKv, kFeedforwardKa,
      kFeedforwardDtSeconds);

  // Trapezoid profile for feedforward
  private final TrapezoidProfile elevatorTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      kMaxVelocity, kMaxAcceleration));

  // Trapezoid profile states
  private TrapezoidProfile.State startState = new TrapezoidProfile.State();
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State();

  // Timer for trapezoid profile
  private final Timer timer = new Timer();

  private double currentSetpoint = 0.0;

  public Elevator() {

    // Set motor configurations
    elevatorMotorConfig.MotorOutput
      .withInverted(kMotorInverted)
      .withNeutralMode(kMotorIdleMode);

    elevatorMotorConfig.CurrentLimits
      .withStatorCurrentLimit(kCurrentLimit);

      //Missing feedback sensor, izone, imaxaccum, outputrange
    elevatorMotorConfig.Slot0
      .withKP(kP)
      .withKI(kI)
      .withKD(kD);
    
      //Missing encoder position conversion factor, velocity conversion factor, zero offset

    // Timer start
    timer.start();

    // Apply the motor configurations, set PID to slot 0
    elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
  }

  public void setMotorVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);
  }

  public void setElevatorSetpoint(double setpoint) {
    currentSetpoint = setpoint;
    timer.reset();
    startState = new TrapezoidProfile.State(elevatorMotor.getPosition().getValueAsDouble(), elevatorMotor.getVelocity().getValueAsDouble());
    goalState = new TrapezoidProfile.State(setpoint, 0.0);
  }

  // Set pivot output based on position, velocity
  public void setMotorOutput(double position, double velocity) {
    // Units might be messed up
    currentState = elevatorTrapezoidProfile.calculate(0.02, currentState, goalState);
    PositionVoltage request = new PositionVoltage(0).withSlot(0)
      .withPosition(currentState.position)
      .withVelocity(currentState.velocity);

    elevatorMotor.setControl(request);
  }

  // Returns true if limit switches are pressed
  public boolean getLimitSwitchTop() {
    return elevatorLimitSwitchTop.get();
  }

  public boolean getLimitSwitchBottom() {
    return elevatorLimitSwitchBottom.get();
  }

  // Set motor voltage to zero, triggered by limit switches in RobotContainer
  public Command elevatorAtTopLimit() {
    return this.run(() -> setMotorVoltage(0.0)).withName("Elevator At Top Limit");
  }

  // Zero encoders and lift elevator slightly, triggered by limit switches in RobotContainer
  public Command elevatorAtBottomLimit() {
    return this.startRun(() -> zeroEncoder(), () -> setMotorVoltage(1.0)).withName("Elevator At Bottom Limit");
  }

  // Default command - hold position
  public Command holdState() {
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

  // Commands to go to various pre-defined positions
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

  public Command goToFeed() {
    return goToPosition(kSetpointFeed, "Feed");
  }

  public Command goToStow() {
    return goToPosition(kSetpointStow, "Stow");
  }

  public void zeroEncoder() {
    elevatorMotor.setPosition(0.0);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder); // Not sure why we need this

    // Motor information
    builder.addDoubleProperty("Elevator Motor Temperature", () -> elevatorMotor.getDeviceTemp().getValueAsDouble(), null);
    builder.addDoubleProperty("Elevator Motor Closed Loop Output", () -> elevatorMotor.getClosedLoopOutput().getValueAsDouble(), null);
    builder.addDoubleProperty("Elevator Motor Output Current", () -> elevatorMotor.getSupplyCurrent().getValueAsDouble(), null);

    // Add SparkMax information (not done yet)

    // Feedforward values
    // There doesn't seem to be any setters for these. Don't think these would be
    // useful without that so commenting them out for now
    /*
     * builder.addDoubleProperty("Elevator Feedforward Ks", () ->
     * feedforward.getKs(), null);
     * builder.addDoubleProperty("Elevator Feedforward Kg", () ->
     * feedforward.getKg(), null);
     * builder.addDoubleProperty("Elevator Feedforward Kv", () ->
     * feedforward.getKv(), null);
     * builder.addDoubleProperty("Elevator Feedforward Ka", () ->
     * feedforward.getKa(), null);
     */

  }

}