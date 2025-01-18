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
import frc.robot.Constants.GroundAlgaeWheelsConstants;

public class GroundAlgaeWheels extends SubsystemBase {

    private final SparkMax groundAlgaeWheelsMotor = new SparkMax(GroundAlgaeWheelsConstants.kMotorId, MotorType.kBrushless);

    // Photogate (beam break)
    private final DigitalInput groundAlgaeWheelsPhotogate = new DigitalInput(GroundAlgaeWheelsConstants.kPhotogateId);

    public GroundAlgaeWheels() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
