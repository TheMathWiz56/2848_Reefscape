//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private final SparkMax elevatorMotor1 = new SparkMax(ElevatorConstants.kElevatorMotor1Id, MotorType.kBrushless);
  private final SparkMax elevatorMotor2 = new SparkMax(ElevatorConstants.kElevatorMotor2Id, MotorType.kBrushless);

  // LaserCAN
  LaserCan laserCan = new LaserCan(ElevatorConstants.kElevatorLaserCanId);

  // Limit switches
  private final DigitalInput limitSwitchTop = new DigitalInput(ElevatorConstants.kElevatorLimitSwitchTopId);
  private final DigitalInput limitSwitchBottom = new DigitalInput(ElevatorConstants.kElevatorLimitSwitchBottomId);

  private PIDController elevatorPid = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI,
      ElevatorConstants.kElevatorD);

  public Elevator() {
    elevatorPid.setSetpoint(ElevatorConstants.kElevatorSetpointStow);
  }

  public void setMotors(double motor1, double motor2) {
    elevatorMotor1.set(motor1);
    elevatorMotor2.set(motor2);
  }

  public void setMotors(double motors) {
    setMotors(motors, motors);
  }

  public void setSetpoint(double setpoint) {
    elevatorPid.setSetpoint(setpoint);
  }
  
  // Set motor speeds based on PID calculation
  public void motorsPeriodic() {
    double distance = getLaserDistance();
    if(distance != -1.0) 
      setMotors(elevatorPid.calculate(distance));
    else
      setMotors(0.0); //Failsafe if getMeasurement() in getLaserDistance() returns null
  }

  // Returns milimeters
  // According to the documentation getMeasurement() can return null. May need a better way to handle that
  public double getLaserDistance() {
    Measurement measurement = laserCan.getMeasurement();
    if (measurement != null)
      return measurement.distance_mm;
    else
      return -1.0;
  }

  @Override
  public void periodic() {
    motorsPeriodic();
  }

  @Override
  public void simulationPeriodic() {

  }
}