//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.GroundAlgaeWheelsConstants.*;

public class GroundAlgaeWheels extends SubsystemBase {

    private final SparkMax wheelsMotor = new SparkMax(kMotorId, MotorType.kBrushless);
    private final SparkMaxConfig wheelsMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController wheelsMotorContoller = wheelsMotor.getClosedLoopController();

    // Photogate (beam break)
    private final DigitalInput wheelsPhotogate = new DigitalInput(kPhotogateId);

    public GroundAlgaeWheels() {

        // Spark motor configuration
        wheelsMotorConfig
                .inverted(kMotorInverted)
                .idleMode(kMotorIdleMode)
                .smartCurrentLimit(kCurrentLimit);

        //Probably don't need these
        /*
        wheelsMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(kP, kI, kD)
                .outputRange(-1, 1);

        wheelsMotorConfig.absoluteEncoder
            .zeroOffset(kMotorEncoderOffset);
         */

        //Configure the motor
        wheelsMotor.configure(wheelsMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

    public boolean hasAlgae() {
        return !wheelsPhotogate.get();
    }

    public Command intake() {
        return runOnce(() -> wheelsMotor.set(kIntakeSpeed));
    }

    public Command exhaust() {
        return runOnce(() -> wheelsMotor.set(kExhaustSpeed));
    }

    public Command stop() {
        return runOnce(() -> wheelsMotor.stopMotor());
    }

    public Command holdState(){
        return Commands.idle(this);
    }

    public Command outtakeAlgae() {
        return new SequentialCommandGroup(
            exhaust(),
            Commands.waitUntil(() -> !hasAlgae()),
            stop()
        );
    }
    
    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
