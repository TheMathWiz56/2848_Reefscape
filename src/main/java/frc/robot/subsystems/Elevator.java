//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

/*
 * Joseph Notes
 * - [Done] one of the elevator motors should be configured to follow the other, not just setting them both to the same output
 * - [Half finished] If we decide to use the internal encoder or an absolute encoder to measure the elevator's position we can use the internal
 * pid controller on the elevator. 
 * - [Done for the LaserCan setup] Should also have a feedforward controller for the elevator, results in smoothing motion since it allows you to create a motion
 * profile. Look at the arm branch for some ideas of how to imlpement feedforward, I can also help.
 * - [Added 2 commands] You can either use methods like public void setMotors ()... or you can use public Command setMotors() with command
 * factories to simplify code and remove boiler plate code
 * - Add a sendable builder and put all sensor information, motor outputs, setpoints, setpoints errors, PID outputs, Feedforward outputs etc.
 * in periodic send the sendable object to the dashboard for debugging and logging
 */

public class Elevator extends SubsystemBase {

  private final SparkMax elevatorMotor1 = new SparkMax(ElevatorConstants.kElevatorMotor1Id, MotorType.kBrushless);
  private final SparkMax elevatorMotor2 = new SparkMax(ElevatorConstants.kElevatorMotor2Id, MotorType.kBrushless);

  // LaserCAN
  LaserCan laserCan = new LaserCan(ElevatorConstants.kElevatorLaserCanId);

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
  private final DigitalInput elevatorLimitSwitchTop = new DigitalInput(ElevatorConstants.kElevatorLimitSwitchTopId);
  private final DigitalInput elevatorLimitSwitchBottom = new DigitalInput(
      ElevatorConstants.kElevatorLimitSwitchBottomId);

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kElevatorKs,
      ElevatorConstants.kElevatorKg, ElevatorConstants.kElevatorKv, ElevatorConstants.kElevatorKa,
      ElevatorConstants.kElevatorDtSeconds);

  // Spark controllers
  private final SparkClosedLoopController elevatorMotor1Controller = elevatorMotor1.getClosedLoopController();
  private final SparkClosedLoopController elevatorMotor2Controller = elevatorMotor2.getClosedLoopController();

  // Motor configurations
  private final SparkMaxConfig elevatorMotor1Config = new SparkMaxConfig();
  private final SparkMaxConfig elevatorMotor2Config = new SparkMaxConfig();

  // Trapezoid profile for feedforward
  private final TrapezoidProfile elevatorTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      ElevatorConstants.kElevatorMaxVelocity, ElevatorConstants.kElevatorMaxAcceleration));

  // LaserCan controller
  // ProfiledPIDController reference - should handle the TrapezoidProfile
  // functions automatically
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html
  private ProfiledPIDController elevatorPIDLaserCan = new ProfiledPIDController(ElevatorConstants.kElevatorP,
      ElevatorConstants.kElevatorI,
      ElevatorConstants.kElevatorD,
      new TrapezoidProfile.Constraints(
          ElevatorConstants.kElevatorMaxVelocity, ElevatorConstants.kElevatorMaxAcceleration));

  // Encoder position setpoint for Spark PID
  private double elevatorSetpoint = ElevatorConstants.kElevatorSetpointStow;

  public Elevator() {
    // Set LaserCan PID intial position
    elevatorPIDLaserCan.setGoal(ElevatorConstants.kElevatorSetpointStow);

    // Set motor configurations
    elevatorMotor1Config
        .inverted(ElevatorConstants.kElevatorMotor1Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit);

    elevatorMotor1Config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI,
            ElevatorConstants.kElevatorD)
        .iZone(ElevatorConstants.kElevatorIZone)
        .iMaxAccum(ElevatorConstants.kElevatorIMaxAccum)
        .outputRange(-1, 1);

    elevatorMotor1Config.absoluteEncoder
        .zeroOffset(0);

    elevatorMotor2Config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit);

    elevatorMotor2Config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI,
            ElevatorConstants.kElevatorD)
        .iZone(ElevatorConstants.kElevatorIZone)
        .iMaxAccum(ElevatorConstants.kElevatorIMaxAccum)
        .outputRange(-1, 1);

    elevatorMotor2Config.absoluteEncoder
        .zeroOffset(0);

    elevatorMotor2Config.follow(ElevatorConstants.kElevatorMotor1Id, ElevatorConstants.kElevatorMotor2Inverted);

    // Apply the motor configurations
    elevatorMotor1.configure(elevatorMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Timer.delay(0.25); // Allow the configs to be burned in
  }

  public void setMotorVoltage(double voltage) {
    elevatorMotor1.setVoltage(voltage);
  }

  public void setElevatorSetpoint(double setpoint) {
    if (ElevatorConstants.kElevatorUseLaserCan) {
      elevatorPIDLaserCan.setGoal(setpoint);
    } else {
      elevatorSetpoint = setpoint;
    }
  }

  // Set motor speeds based on PID calculation
  // Better named might be holdPosition
  public void holdPosition() {
    if (ElevatorConstants.kElevatorUseLaserCan) {
      double distance = getLaserDistance();
      if (distance != -1.0) // good idea, may have some weird effects though
        setMotorVoltage(elevatorPIDLaserCan.calculate(distance)
            + feedforward.calculate(elevatorPIDLaserCan.getSetpoint().velocity));
      else
        setMotorVoltage(0.0);
    } else {
      // Would probably need to make a full TrapezoidalProfile here
    }
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

  public boolean getLimitSwitches() {
    return elevatorLimitSwitchBottom.get() || elevatorLimitSwitchBottom.get();
  }

  public Command elevatorDefaultCommand() {
    return this.run(() -> holdPosition());
  }

  public Command elevatorAtHardLimit() {
    return this.run(() -> setMotorVoltage(0.0));
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

    if (ElevatorConstants.kElevatorUseLaserCan) {
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