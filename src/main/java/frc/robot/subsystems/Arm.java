package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Arm extends SubsystemBase{

    private final CANSparkMax pivot_motor;
    private final DutyCycleEncoder encoder;

    private double setpoint;

    public Arm (){
        pivot_motor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(0);

        setpoint = 0;
        encoder.initSendable(null);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
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
    }

    public Command go_to_setpoint(double setpoint){
        return runOnce(()->this.setpoint = setpoint);
    }
}