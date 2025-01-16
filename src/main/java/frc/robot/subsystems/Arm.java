//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    private final SparkMax armPivotMotor = new SparkMax(ArmConstants.kArmPivotMotorId, MotorType.kBrushless);
    private final SparkMax armIntakeMotor = new SparkMax(ArmConstants.kArmWheelsMotorId, MotorType.kBrushless);

    // Photogate (beam break)
    private final DigitalInput photogate = new DigitalInput(ArmConstants.kArmPhotogateId);

    // ABS Encoder - could be an AnalogInput instead
    private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(ArmConstants.kArmAbsEncoderId);

    private PIDController armPid = new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD);

    // Potentially also 2 limit switches - DigitalInput class
    // There will also be a USB camera that I don't think will be represented here

    

    public Arm() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
