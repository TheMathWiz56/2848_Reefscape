//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
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
 * - [Done] If we decide to use the internal encoder or an absolute encoder to measure the elevator's position we can use the internal
 * pid controller on the elevator. 
 * - Should also have a feedforward controller for the elevator, results in smoothing motion since it allows you to create a motion
 * profile. Look at the arm branch for some ideas of how to imlpement feedforward, I can also help.
 * - You can either use methods like public void setMotors ()... or you can use public Command setMotors() with command
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
   * Should be bound to a trigger in robotcontainer that calls something like
   * public Command atHardLimit()
   * Which 0's the motor output if it is trying to drive the elevator into an
   * unsafe state or allows the elevator to move
   * if the motors are trying to move it to a safe state.
   * ie. we are at the Bottom limit switch and trying to drive the elevator down
   * more, don't allow motor output
   * if we are at the bottom limit switch and tring to drive the elevator up,
   * allow motor output
   */
  private final DigitalInput limitSwitchTop = new DigitalInput(ElevatorConstants.kElevatorLimitSwitchTopId);
  private final DigitalInput limitSwitchBottom = new DigitalInput(ElevatorConstants.kElevatorLimitSwitchBottomId);

  /*
   * private PIDController elevatorPid = new
   * PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI,
   * ElevatorConstants.kElevatorD);
   */
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kElevatorKs,
      ElevatorConstants.kElevatorKg, ElevatorConstants.kElevatorKv, ElevatorConstants.kElevatorKa,
      ElevatorConstants.kElevatorDtSeconds);

  //Motor configurations
  private final SparkMaxConfig elevatorMotor1Config = new SparkMaxConfig();
  private final SparkMaxConfig elevatorMotor2Config = new SparkMaxConfig();

  public Elevator() {
    // Set motor configurations
    elevatorMotor1Config
        .inverted(ElevatorConstants.kElevatorMotor1Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);

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
        .smartCurrentLimit(40);
    
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

  public void setMotors(double motor1, double motor2) {
    elevatorMotor1.set(motor1);
  }

  /*
   * public void setSetpoint(double setpoint) {
   * elevatorPid.setSetpoint(setpoint);
   * }
   */

  // Set motor speeds based on PID calculation
  // Better named might be holdPosition
  public void holdPosition() {
    double distance = getLaserDistance();
    // if(distance != -1.0) // good idea, may have some weird effects though
    // setMotors(elevatorPid.calculate(distance));
    // else
    // setMotors(0.0); //Failsafe if getMeasurement() in getLaserDistance() returns
    // null
  }

  // Returns milimeters
  // According to the documentation getMeasurement() can return null. May need a
  // better way to handle that
  public double getLaserDistance() {
    Measurement measurement = laserCan.getMeasurement();
    if (measurement != null)
      return measurement.distance_mm;
    else
      return -1.0;
  }

  @Override
  public void periodic() {
    holdPosition();
  }

  @Override
  public void simulationPeriodic() {

  }
}