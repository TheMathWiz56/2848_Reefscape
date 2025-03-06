//Subsystem template

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.AscenderConstants.*;

import java.util.function.DoubleSupplier;

public class Ascender extends SubsystemBase {
    
    private final SparkMax ascenderMotor = new SparkMax(kAscenderMotorId, SparkMax.MotorType.kBrushless);

    // On the bottom
    private final DigitalInput ascenderLimitSwitch = new DigitalInput(kAscenderLimitSwitchId);

    public Ascender() {

    }

    /*
    public Command climb() {
        return run(() -> ascenderMotor.set(1)).until(() -> false) // Need a stoping condition here
        .andThen(runEnd(() -> ascenderMotor.set(-1), () -> ascenderMotor.stopMotor()).until(() -> ! ascenderLimitSwitch.get()));
    } */

    public Command manualClimb(DoubleSupplier input) {
        return run(() -> ascenderMotor.set(getAscenderOutput(input)));
    }

    private double getAscenderOutput(DoubleSupplier input){
      SmartDashboard.putBoolean("Is running condition", input.getAsDouble() < 0 && !ascenderLimitSwitch.get());
      if (input.getAsDouble() < 0 && !ascenderLimitSwitch.get()){
        return 0;
      }
      else{
        SmartDashboard.putNumber("Ascender Output", input.getAsDouble() * 0.5);
        return input.getAsDouble() * 0.5;
      }
    }

    //TODO: fill in start and stop
    public void start(){

    }
    public void stop() {
        
    }

    @Override
    public void periodic() {
      SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {

    }

    
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder); // Not sure why we need this

    // Motor information
    builder.addBooleanProperty("Ascender Limit Switch", () -> !ascenderLimitSwitch.get(), null);
  }

}
