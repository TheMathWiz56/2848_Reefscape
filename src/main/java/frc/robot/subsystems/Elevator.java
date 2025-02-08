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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

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

  // Timer for trapezoid profile
  private final Timer timer = new Timer();

  private double currentSetpoint = 0.0;

  private boolean isZeroed = false;

  public Elevator() {

    // Set motor configurations
    elevatorMotorConfig.MotorOutput
        .withInverted(kMotorInverted)
        .withNeutralMode(kMotorIdleMode);

    elevatorMotorConfig.CurrentLimits
        .withSupplyCurrentLimit(kCurrentLimit);

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

  public boolean getLimitSwitchBottom() {
    return elevatorLimitSwitchBottom.get();
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
    isZeroed = true;
  }

  public Command autoZeroEncoder() {
    return run(() -> setMotorVoltage(1.0)).until(() -> elevatorLimitSwitchBottom.get())
        .andThen(runOnce(() -> zeroEncoder())).withName("Zero Encoder");
  }

  @Override
  public void periodic() {
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
    builder.addDoubleProperty("Profile Current Velocity", () -> currentState.velocity, null);

    builder.addDoubleProperty("Timer", () -> timer.get(), null);

    // Add closed loop information (not done yet)
    builder.addDoubleProperty("Elevator kP,", () -> elevatorMotorConfig.Slot0.kP, (kPNew) -> {
      elevatorMotorConfig.Slot0.withKP(kPNew);
      elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
    });
    builder.addDoubleProperty("Elevator kI,", () -> elevatorMotorConfig.Slot0.kI, (kINew) -> {
      elevatorMotorConfig.Slot0.withKI(kINew);
      elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
    });
    builder.addDoubleProperty("Elevator kD,", () -> elevatorMotorConfig.Slot0.kD, (kDNew) -> {
      elevatorMotorConfig.Slot0.withKD(kDNew);
      elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
    });

    // Feedforward values
    // There doesn't seem to be any setters for these. Don't think these would be
    // useful without that so commenting them out for now

    builder.addDoubleProperty("Elevator Feedforward Ks", () -> feedforward.getKs(), null);
    builder.addDoubleProperty("Elevator Feedforward Kg", () -> feedforward.getKg(), null);
    builder.addDoubleProperty("Elevator Feedforward Kv", () -> feedforward.getKv(), null);
    builder.addDoubleProperty("Elevator Feedforward Ka", () -> feedforward.getKa(), null);

  }

}