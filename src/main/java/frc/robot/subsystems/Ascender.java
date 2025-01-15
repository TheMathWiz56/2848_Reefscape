//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AscenderConstants;

public class Ascender extends SubsystemBase {

    SparkMax ascenderMotor = new SparkMax(AscenderConstants.kAscenderMotorId, SparkMax.MotorType.kBrushless);

    // ABS Encoder - could be an AnalogInput instead
    DutyCycleEncoder absEncoder = new DutyCycleEncoder(AscenderConstants.kAscenderAbsEncoderId);

    // Potentially also 2 limit switches - DigitalInput class
    // Additionally there will be a camera

    public Ascender() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
