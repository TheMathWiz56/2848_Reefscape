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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundAlgaeWheelsConstants;

public class GroundAlgaeWheels extends SubsystemBase {

    private final SparkMax wheelsMotor = new SparkMax(GroundAlgaeWheelsConstants.kMotorId, MotorType.kBrushless);
    private final SparkMaxConfig wheelsMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController wheelsMotorContoller = wheelsMotor.getClosedLoopController();

    // Photogate (beam break)
    private final DigitalInput wheelsPhotogate = new DigitalInput(GroundAlgaeWheelsConstants.kPhotogateId);

    public GroundAlgaeWheels() {

        // Spark motor configuration
        wheelsMotorConfig
                .inverted(GroundAlgaeWheelsConstants.kMotorInverted)
                .idleMode(GroundAlgaeWheelsConstants.kMotorIdleMode)
                .smartCurrentLimit(GroundAlgaeWheelsConstants.kCurrentLimit);

        //Probably don't need these
        /*
        wheelsMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(GroundAlgaeWheelsConstants.kP, GroundAlgaeWheelsConstants.kI, GroundAlgaeWheelsConstants.kD)
                .outputRange(-1, 1);

        wheelsMotorConfig.absoluteEncoder
            .zeroOffset(GroundAlgaeWheelsConstants.kMotorEncoderOffset);
         */

        //Configure the motor
        wheelsMotor.configure(wheelsMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

    public boolean hasAlgae() {
        return !wheelsPhotogate.get();
    }

    public Command intake() {
        return runOnce(() -> wheelsMotor.set(GroundAlgaeWheelsConstants.kIntakeSpeed));
    }

    public Command exhaust() {
        return runOnce(() -> wheelsMotor.set(GroundAlgaeWheelsConstants.kExhaustSpeed));
    }

    public Command stop() {
        return runOnce(() -> wheelsMotor.stopMotor());
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
