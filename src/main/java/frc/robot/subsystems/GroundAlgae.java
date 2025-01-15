//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GroundAlgaeConstants;

public class GroundAlgae extends SubsystemBase {

    private final SparkMax groundAlgaeMotor1 = new SparkMax(GroundAlgaeConstants.kGroundAlgaeMotor1Id, MotorType.kBrushless);
    private final SparkMax groundAlgaeMotor2 = new SparkMax(GroundAlgaeConstants.kGroundAlgaeMotor2Id, MotorType.kBrushless);

    // Photogate (beam break)
    private final DigitalInput photogate = new DigitalInput(GroundAlgaeConstants.kGroundAlgaePhotogateId);

    // ABS Encoder - could be an AnalogInput instead
    private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(GroundAlgaeConstants.kGroundAlgaeAbsEncoderId);

    public GroundAlgae() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
