//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AscenderConstants.*;

// Seems like there is uncertainty about the ascender, so I'm holding off on it for now
public class Ascender extends SubsystemBase {
    
    private final SparkMax ascenderMotor = new SparkMax(kAscenderMotorId, SparkMax.MotorType.kBrushless);

    private final DigitalInput ascenderLimitSwitch = new DigitalInput(kAscenderLimitSwitchId);

    public Ascender() {

    }

    public Command climb() {
        return runEnd(() -> ascenderMotor.set(1), () -> ascenderMotor.stopMotor()).until(() -> ascenderLimitSwitch.get());
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
