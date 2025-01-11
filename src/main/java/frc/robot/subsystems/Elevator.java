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

public class Elevator extends SubsystemBase {
  
    SparkMax elevatorMotor1 = new SparkMax(0, MotorType.kBrushless);
    SparkMax elevatorMotor2 = new SparkMax(0, MotorType.kBrushless);

    //Lidar sensor - could be a serial bus input instead
    AnalogInput lidar = new AnalogInput(0);
    
    //Limit switches
    DigitalInput limitSwitchTop = new DigitalInput(0);
    DigitalInput limitSwitchBottom = new DigitalInput(0); 

  public Elevator() {

  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}