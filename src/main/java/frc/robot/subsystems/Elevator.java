//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

  private final SparkMax elevatorMotor1 = new SparkMax(kMotor1Id, MotorType.kBrushless);
  private final SparkMax elevatorMotor2 = new SparkMax(kMotor2Id, MotorType.kBrushless);

  // LaserCAN
  LaserCan laserCan = new LaserCan(kLaserCanId);

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

  // Spark controllers
  private final SparkClosedLoopController elevatorMotor1Controller = elevatorMotor1.getClosedLoopController();
  private final SparkClosedLoopController elevatorMotor2Controller = elevatorMotor2.getClosedLoopController();

  // Spark ABS encoders
  private final RelativeEncoder elevatorMotor1Encoder = elevatorMotor1.getEncoder();
  private final RelativeEncoder elevatorMotor2Encoder = elevatorMotor2.getEncoder();

  // Motor configurations
  private final SparkMaxConfig elevatorMotor1Config = new SparkMaxConfig();
  private final SparkMaxConfig elevatorMotor2Config = new SparkMaxConfig();

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

  // LaserCan controller
  // ProfiledPIDController reference - should handle the TrapezoidProfile
  // functions automatically
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html
  private ProfiledPIDController elevatorPIDLaserCan = new ProfiledPIDController(kPLaserCan,
      kILaserCan,
      kDLaserCan,
      new TrapezoidProfile.Constraints(
          kMaxVelocity, kMaxAcceleration));

  public Elevator() {
    // Set LaserCan PID intial position
    elevatorPIDLaserCan.setGoal(kSetpointStow);

    // Set motor configurations
    elevatorMotor1Config
        .inverted(kMotor1Inverted)
        .idleMode(kMotorIdleMode)
        .smartCurrentLimit(kCurrentLimit);

    elevatorMotor1Config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(kP, kI,
            kD)
        .iZone(kIZone)
        .iMaxAccum(kIMaxAccum)
        .outputRange(-1, 1);

    elevatorMotor1Config.encoder
        .positionConversionFactor(kMotorPositionConversionFactor)
        .velocityConversionFactor(kMotorVelocityConversionFactor);

    elevatorMotor1Config.absoluteEncoder
        .zeroOffset(kMotor1EncoderOffset);

    elevatorMotor2Config
        .idleMode(kMotorIdleMode)
        .smartCurrentLimit(kCurrentLimit);

    elevatorMotor2Config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(kP, kI,
            kD)
        .iZone(kIZone)
        .iMaxAccum(kIMaxAccum)
        .outputRange(-1, 1);

    elevatorMotor2Config.encoder
        .positionConversionFactor(kMotorPositionConversionFactor)
        .velocityConversionFactor(kMotorVelocityConversionFactor);

    elevatorMotor2Config.absoluteEncoder
        .zeroOffset(kMotor2EncoderOffset);

    elevatorMotor2Config.follow(kMotor1Id, kMotor2Inverted);

    // Timer start
    timer.start();

    // Apply the motor configurations
    elevatorMotor1.configure(elevatorMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setMotorVoltage(double voltage) {
    elevatorMotor1.setVoltage(voltage);
  }

  public void setElevatorSetpoint(double setpoint) {
    currentSetpoint = setpoint;
    if (kUseLaserCan) {
      elevatorPIDLaserCan.setGoal(setpoint);
    } else {
      timer.reset();
      startState = new TrapezoidProfile.State(elevatorMotor1Encoder.getPosition(), elevatorMotor1Encoder.getVelocity());
      goalState = new TrapezoidProfile.State(setpoint, 0.0);
    }
  }

  // Set motor speeds based on PID calculation (LaserCan)
  public void setMotorSpeedsLaserCan() {
    if (kUseLaserCan) {
      double distance = getLaserDistance();
      if (distance != -1.0) // good idea, may have some weird effects though
        setMotorVoltage(elevatorPIDLaserCan.calculate(distance)
            + feedforward.calculate(elevatorPIDLaserCan.getSetpoint().velocity));
      else
        setMotorVoltage(0.0);
    }
  }

  // Set pivot output based on position, velocity (without LaserCan)
  public void setMotorOutput(double position, double velocity) {
    // Units probably messed up
    elevatorMotor1Controller.setReference(currentState.position, SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0, feedforward.calculate(currentState.position, currentState.velocity));
  }

  // Returns meters
  // According to the documentation getMeasurement() can return null. May need a
  // better way to handle that
  public double getLaserDistance() {
    Measurement measurement = laserCan.getMeasurement();
    if (measurement != null)
      return measurement.distance_mm * 1000;
    else
      return -1.0;
  }

  // Returns true if either limit switch is pressed
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

  // Set motor voltage to zero, triggered by limit switches in RobotContainer
  public Command elevatorAtBottomLimit() {
    return this.startRun(() -> zeroEncoders(), () -> setMotorVoltage(1.0)).withName("Elevator At Bottom Limit");
  }

  // Default command - hold position
  public Command holdState() {
    return this.run(() -> {
      if(kUseLaserCan) setMotorSpeedsLaserCan();
      else setMotorOutput(currentSetpoint, 0.0);
    }).withName("Elevator Default Command");
  }


  // Command to go to position
  public Command goToPosition(double position, String positionName) {
    return this.startRun(() -> {
      setElevatorSetpoint(position);
    }, () -> {
      if (kUseLaserCan)
        setMotorSpeedsLaserCan();
      else {
        currentState = elevatorTrapezoidProfile.calculate(timer.get(), startState, goalState);
        setMotorOutput(currentState.position, currentState.velocity);
      }
    }).until(() -> kUseLaserCan ? elevatorPIDLaserCan.atGoal() : elevatorTrapezoidProfile.isFinished(timer.get()))
        .withName("Go to " + positionName);
  }

  // Commands to go to various pre-defined positions
  public Command goToL1() {
    return goToPosition(kUseLaserCan ? kSetpointL1LaserCan : kSetpointL1, "L1");
  }

  public Command goToL2() {
    return goToPosition(kUseLaserCan ? kSetpointL2LaserCan : kSetpointL2, "L2");
  }

  public Command goToL3() {
    return goToPosition(kUseLaserCan ? kSetpointL3LaserCan : kSetpointL3, "L3");
  }

  public Command goToL4() {
    return goToPosition(kUseLaserCan ? kSetpointL4LaserCan : kSetpointL4, "L4");
  }

  public Command goToFeed() {
    return goToPosition(kUseLaserCan ? kSetpointFeedLaserCan : kSetpointFeed, "Feed");
  }

  public Command goToStow() {
    return goToPosition(kUseLaserCan ? kSetpointStowLaserCan : kSetpointStow, "Stow");
  }

  public void zeroEncoders() {
    elevatorMotor1Encoder.setPosition(0.0);
    elevatorMotor2Encoder.setPosition(0.0);
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
    builder.addDoubleProperty("Elevator Motor 1 Temperature", () -> elevatorMotor1.getMotorTemperature(), null);
    builder.addDoubleProperty("Elevator Motor 1 Output", () -> elevatorMotor1.getAppliedOutput(), null);
    builder.addDoubleProperty("Elevator Motor 1 Output Current", () -> elevatorMotor1.getOutputCurrent(), null);

    builder.addDoubleProperty("Elevator Motor 2 Temperature", () -> elevatorMotor2.getMotorTemperature(), null);
    builder.addDoubleProperty("Elevator Motor 2 Output", () -> elevatorMotor2.getAppliedOutput(), null);
    builder.addDoubleProperty("Elevator Motor 2 Output Current", () -> elevatorMotor2.getOutputCurrent(), null);

    if (kUseLaserCan) {
      // LaserCan distance
      double distance = getLaserDistance();
      builder.addDoubleProperty("Elevator LaserCan Distance", () -> distance, null);

      // Setpoint and goal positions, velocities
      builder.addDoubleProperty("Elevator Setpoint Position",
          () -> elevatorPIDLaserCan.getSetpoint().position,
          null);
      builder.addDoubleProperty("Elevator Setpoint Velocity",
          () -> elevatorPIDLaserCan.getSetpoint().velocity,
          null);
      builder.addDoubleProperty("Elevator Goal Position",
          () -> elevatorPIDLaserCan.getGoal().position,
          null);
      builder.addDoubleProperty("Elevator Goal Velocity",
          () -> elevatorPIDLaserCan.getGoal().velocity,
          null);

      // PID values
      builder.addDoubleProperty("Elevator kP", () -> elevatorPIDLaserCan.getP(),
          value -> elevatorPIDLaserCan.setP(value));
      builder.addDoubleProperty("Elevator kI", () -> elevatorPIDLaserCan.getI(),
          value -> elevatorPIDLaserCan.setI(value));
      builder.addDoubleProperty("Elevator kD", () -> elevatorPIDLaserCan.getD(),
          value -> elevatorPIDLaserCan.setD(value));

      // PID output
      builder.addDoubleProperty("Elevator PID Output",
          () -> distance != -1 ? elevatorPIDLaserCan.calculate(getLaserDistance()) : -1,
          null);

      // Feedforward output
      builder.addDoubleProperty("Elevator Feedforward Output",
          () -> feedforward.calculate(elevatorPIDLaserCan.getSetpoint().velocity), null);
    } else {
      // Add SparkMax information (not done yet)
    }

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