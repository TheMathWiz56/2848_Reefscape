package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Arm extends SubsystemBase{

    private final CANSparkMax pivot_motor;
    private final SparkPIDController pivot_controller;
    private final DutyCycleEncoder encoder;
    private final AbsoluteEncoder rev_encoder;

    private final ArmFeedforward feedforward;

    private double setpoint, P, I, D;

    public Arm (){
        pivot_motor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(0);
        rev_encoder = encoder;

        setpoint = 0;
        P = 0;
        I = 0;
        D = 0;
        pivot_controller = pivot_motor.getPIDController();
        update_controller_PID();
        pivot_controller.setOutputRange(-1, 1);

        feedforward = new ArmFeedforward(0, 0, 0, 0);
    }

    @Override
    public void periodic() {
        if (is_PID_updated()){
            update_controller_PID();
        }
        
        pivot_controller.setReference(setpoint, CANSparkBase.ControlType.kPosition);
        pivot_controller.setFeedbackDevice(encoder);
        pivot_controller.setFF(I);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This might be necessary to have the sendable builder work
        //super.initSendable(builder);
        
        builder.addDoubleProperty("setpoint", ()->setpoint, null);
        // Might also be able to add the sendable builder for the encoder to this: check later
        builder.addDoubleProperty("position", ()->encoder.getAbsolutePosition(), null);

        builder.addDoubleProperty("P", null, null);
        builder.addDoubleProperty("I", null, null);
        builder.addDoubleProperty("D", null, null);
    }

    public Command go_to_setpoint(double setpoint){
        return runOnce(()->this.setpoint = setpoint);
    }

    private void update_controller_PID(){
        pivot_controller.setP(P);
        pivot_controller.setI(I);
        pivot_controller.setD(D);
    }

    private boolean is_PID_updated() {
        return pivot_controller.getP() != P || 
               pivot_controller.getI() != I || 
               pivot_controller.getD() != D;
    }
}