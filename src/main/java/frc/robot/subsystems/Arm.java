package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class Arm extends SubsystemBase{

    private final CANSparkMax pivot_motor;
    private final SparkPIDController pivot_controller;
    private final AbsoluteEncoder abs_encoder;

    private final ArmFeedforward feedforward;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State current_setpoint;

    // Reference is the same as a setpoint, but for clarity, the final target arm position is reference
    private double reference, previous_reference, P, I, D, FF;

    // Timer for stepping between motion profile setpoints
    private final Timer timer = new Timer();

    // Define a small tolerance for floating-point comparison. 
    // (JOSEPH) SHOULD BE PLACED IN CONSTANTS FILE ONCE MERGED, also should place the double comparison in utils ...
    private final double EPSILON = 1e-9;

    public Arm (){
        pivot_motor = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);
        pivot_motor.restoreFactoryDefaults();
        pivot_motor.setSmartCurrentLimit(40);
        pivot_motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        pivot_motor.burnFlash();
        // Might need a delay

        abs_encoder = pivot_motor.getAbsoluteEncoder();
        // 19 deg -(0.0527777) + 0.1745584
        abs_encoder.setZeroOffset(0.12107807);

        reference = 0;
        previous_reference = 0;
        current_setpoint = new TrapezoidProfile.State();
        P = 0; //1.5
        I = 0;
        D = 0;
        FF = 0;

        pivot_controller = pivot_motor.getPIDController();
        update_controller_PID();

        pivot_controller.setOutputRange(-1, 1);
        pivot_controller.setFeedbackDevice(abs_encoder);

        // Gains from ReCalc, either experiment or use sysID to determien ks
        feedforward = new ArmFeedforward(0, 0.07, 1.25, 0);

        // Rev/s and Rev/s/s         |        1.33 rev/s and ~1.33 rev/s/s MAX
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5,1));

        // Start at subsystem initialization
        timer.start();
    }

    @Override
    public void periodic() {
        if (is_PID_updated()){
            update_controller_PID();
        }
        
        if (is_reference_updated()){
            // Restart motion profile timer
            timer.reset();
        }
        
        // Calculates the position and velocity for the profile at a time t where the current state is at time t = 0.
        current_setpoint = profile.calculate(timer.get(), new TrapezoidProfile.State(abs_encoder.getPosition(), abs_encoder.getVelocity()),
                                                     new TrapezoidProfile.State(reference, 0));
        
        // Calculates the feedforward using the position for the kG and velocity setpoint for kV. MIGHT need to divide by the battery voltage
        FF = feedforward.calculate(abs_encoder.getPosition(), current_setpoint.velocity);
        
        // Feed the PID the current position setpoint from the motion profile
        //pivot_controller.setReference(current_setpoint.position, CANSparkBase.ControlType.kPosition); // Make sure updating the Spark Max reference doesn't reset the I accum
        //pivot_controller.setFF(FF);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This might be necessary to have the sendable builder work
        //super.initSendable(builder);
        
        builder.addDoubleProperty("reference", ()->reference, value->reference = value);
        // Might also be able to add the sendable builder for the encoder to this: check later
        builder.addDoubleProperty("position", ()->abs_encoder.getPosition(), null);

        builder.addDoubleProperty("P", null, null);
        builder.addDoubleProperty("I", null, null);
        builder.addDoubleProperty("D", null, null);

        builder.addDoubleProperty("timer", ()-> timer.get(), null);
        builder.addDoubleProperty("FF", ()->FF, null);
        builder.addDoubleProperty("MP_Position", ()->current_setpoint.position, null);
        builder.addDoubleProperty("MP_Velocity", ()->current_setpoint.velocity, null);
    }

    public Command go_to_reference(double reference){
        return runOnce(()-> this.reference = reference);
    }

    /**
     * Updates the PID controller with the current values of P, I, and D.
     */
    private void update_controller_PID() {
        // Set the proportional gain (P) of the PID controller
        pivot_controller.setP(P);
        // Set the integral gain (I) of the PID controller
        pivot_controller.setI(I);
        // Set the derivative gain (D) of the PID controller
        pivot_controller.setD(D);
    }

    /**
     * Checks if the PID gains (P, I, D) in the controller are up-to-date 
     * with the current values. 
     * 
     * @return true if any of the PID gains in the controller are 
     *         different from the current values, false otherwise.
     */
    private boolean is_PID_updated() {
        // Compare the proportional gain (P) in the controller with the current value
        // Compare the integral gain (I) in the controller with the current value
        // Compare the derivative gain (D) in the controller with the current value
        // Return true if any of these gains differ
        return pivot_controller.getP() != P || 
            pivot_controller.getI() != I || 
            pivot_controller.getD() != D;
    }

    /**
     * Checks if the current setpoint has significantly changed from the previous setpoint.
     *
     * @return true if the setpoint has changed, false otherwise.
     */
    private boolean is_reference_updated() {
        // Check if the setpoint has changed significantly
        if (Math.abs(reference - previous_reference) > EPSILON) {
            return true; // A significant update has occurred
        }
        return false; // No significant change
    }
}