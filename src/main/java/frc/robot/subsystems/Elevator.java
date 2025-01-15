//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  
    SparkMax elevatorMotor1 = new SparkMax(ElevatorConstants.kElevatorMotor1Id, MotorType.kBrushless);
    SparkMax elevatorMotor2 = new SparkMax(ElevatorConstants.kElevatorMotor2Id, MotorType.kBrushless);

    //Lidar sensor - could be a serial bus input instead
    AnalogInput lidar = new AnalogInput(ElevatorConstants.kElevatorLidarId);
    
    
    //Limit switches
    DigitalInput limitSwitchTop = new DigitalInput(ElevatorConstants.kElevatorLimitSwitchTopId);
    DigitalInput limitSwitchBottom = new DigitalInput(ElevatorConstants.kElevatorLimitSwitchBottomId); 

  public Elevator() {

  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}