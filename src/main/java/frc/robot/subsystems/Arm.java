package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Represents the arm subsystem, which controls the pivot motion of an arm mechanism.
 * This class includes configurations for motion control, PID updates, and integration with WPILib tools.
 */
public class Arm extends SubsystemBase {

    // Motor controller and configuration objects
    private final SparkMax pivot_motor; // Motor controlling the arm
    private final SparkMaxConfig pivot_config; // Configuration for the motor
    private final SparkClosedLoopController pivot_controller; // Controller to manage closed-loop PID control
    private final AbsoluteEncoder abs_encoder; // Encoder to measure the position of the arm

    // Feedforward controller for arm motion (helps to predict required motor output)
    private final ArmFeedforward feedforward;

    // Motion profiling and state tracking
    private final TrapezoidProfile profile; // Profile for trapezoidal motion
    private TrapezoidProfile.State goal_state, start_state, current_state; // States used for motion control

    // PID and feedforward gains (tuned for specific motion behavior)
    private double reference, previous_reference, P, I, D, FF, IZone, IMaxAccum;
    private boolean PIDupdated; // Flag to track if PID parameters need to be updated

    // Timer for motion profile timing
    private final Timer timer = new Timer();

    // System identification routine (used for tuning and testing the system)
    private final SysIdRoutine routine;

    // Tolerance for floating-point comparison (helps avoid small errors)
    private static final double EPSILON = 1e-9;

    /**
     * Constructor for the Arm subsystem. Initializes the motor, encoders, feedforward, 
     * and other configuration components.
     */
    public Arm() {
        // Initializing motor and configuration objects
        pivot_motor = new SparkMax(13, SparkLowLevel.MotorType.kBrushless); // Motor on channel 13 (Brushless)
        pivot_config = new SparkMaxConfig(); // Default configuration for the motor
        abs_encoder = pivot_motor.getAbsoluteEncoder(); // Encoder attached to the motor
        pivot_controller = pivot_motor.getClosedLoopController(); // Closed-loop controller for controlling motor output

        // Initial setpoints and PID gains
        reference = 0.25; // Default reference position for the arm (scaled value [0,1])
        previous_reference = 0.25; // Initial previous reference value for comparison
        goal_state = new TrapezoidProfile.State(); // Trapezoid profile goal state (target position)
        start_state = new TrapezoidProfile.State(); // Starting state for trapezoidal motion
        current_state = new TrapezoidProfile.State(); // Current state during motion

        // PID gains
        P = 7; // Proportional gain
        I = 0; // Integral gain
        IZone = 0; // Integral zone
        IMaxAccum = 0; // Max accumulation for integral term
        D = 1.233; // Derivative gain
        FF = 0; // Feedforward term (helps to predict motor output)
        PIDupdated = false; // Flag to track if PID parameters need updating

        // Configure the pivot motor
        pivot_config
            .inverted(false) // Motor is not inverted
            .idleMode(IdleMode.kCoast) // Motor will coast when not powered
            .smartCurrentLimit(40); // Set current limit to 40 amps to protect the motor

        pivot_config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // Using the absolute encoder as feedback
            .pid(P, I, D) // Set PID gains
            .iZone(IZone) // Set integral zone
            .iMaxAccum(IMaxAccum) // Set max accumulation for integral term
            .outputRange(-1, 1); // Set output range to [-1, 1]

        // Absolute encoder configuration
        pivot_config.absoluteEncoder
            .zeroOffset(0.12107807); // Zero offset for the encoder (calibration value)

        // Arm feedforward configuration (units: percent output)
        feedforward = new ArmFeedforward(0, 0.035173, 1.1233, 0.11294); // Feedforward constants (calibrated)

        // Motion profile constraints (units: rev/s and rev/s^2)
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5, 1)); // Speed and acceleration constraints for motion profile

        // Initialize system identification routine (used for tuning and testing)
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(), // Default config for system identification
            new SysIdRoutine.Mechanism(this::voltageDrive, null, this)); // Mechanism for applying voltages

        // Start timer to track motion profile time
        timer.start();

        // Apply the configuration to the motor controller
        pivot_motor.configure(pivot_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Timer.delay(0.1); // Allow configuration to apply before continuing
    }

    /**
     * Creates a command for system identification in quasistatic mode.
     * This mode helps test how the system behaves under slow movement.
     * 
     * @param direction the direction of motion
     * @return the command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction) // Perform quasistatic system identification
            .until(() -> isSoftLimitReached()) // Stop once a soft limit is reached
            .andThen(stop_motor()); // Stop the motor when done
    }

    /**
     * Creates a command for system identification in dynamic mode.
     * This mode tests the system under fast or dynamic movement.
     * 
     * @param direction the direction of motion
     * @return the command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction) // Perform dynamic system identification
            .until(() -> isSoftLimitReached()) // Stop once a soft limit is reached
            .andThen(stop_motor()); // Stop the motor when done
    }

    /**
     * Drives the motor using a voltage input.
     * This is used during system identification or manual control.
     * 
     * @param voltage the voltage to apply
     */
    public void voltageDrive(Voltage voltage) {
        SmartDashboard.putNumber("SysID magnitude", voltage.magnitude()); // Display voltage magnitude on SmartDashboard
        pivot_motor.setVoltage(voltage.magnitude()); // Apply the voltage to the motor
    }

    @Override
    public void periodic() {
        // This is called periodically (every 20ms) during robot operation

        // If PID parameters were updated, apply the changes to the controller
        if (PIDupdated) {
            update_controller_PID(); // Update PID values on the motor controller
            PIDupdated = false; // Reset the flag after updating
        }

        SmartDashboard.putData(this); // Update SmartDashboard with subsystem data
    }

    @Override
    public void simulationPeriodic() {
        // Called during simulation mode (not used in this example)
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder); // Initialize the sendable builder

        // Add properties to the sendable builder to display them on SmartDashboard
        builder.addDoubleProperty("previous reference", () -> previous_reference, null);
        builder.addDoubleProperty("reference", () -> reference, value -> reference = adjusted_reference(value));
        builder.addDoubleProperty("position", () -> abs_encoder.getPosition(), null);

        builder.addDoubleProperty("P", () -> P, value -> { P = value; PIDupdated = true; });
        builder.addDoubleProperty("I", () -> I, value -> { I = value; PIDupdated = true; });
        builder.addDoubleProperty("IZone", () -> IZone, value -> { IZone = value; PIDupdated = true; });
        builder.addDoubleProperty("IMaxAccum", () -> IMaxAccum, value -> { IMaxAccum = value; PIDupdated = true; });
        builder.addDoubleProperty("D", () -> D, value -> { D = value; PIDupdated = true; });

        builder.addDoubleProperty("timer", () -> timer.get(), null);
        builder.addDoubleProperty("FF", () -> FF, null);
        builder.addDoubleProperty("MP_Position", () -> goal_state.position, null);
        builder.addDoubleProperty("MP_Velocity", () -> goal_state.velocity, null);
        builder.addDoubleProperty("motor output", () -> pivot_motor.getAppliedOutput(), null);
        builder.addDoubleProperty("Error", () -> goal_state.position - abs_encoder.getPosition(), null);
        builder.addDoubleProperty("Error_Degrees", () -> (goal_state.position - abs_encoder.getPosition()) * 360, null);

        builder.addBooleanProperty("Reverse Soft Limit", this::isReverseLimitReached, null);
        builder.addBooleanProperty("Forward Soft Limit", this::isForwardLimitReached, null);
        builder.addBooleanProperty("Combined Limits", this::isSoftLimitReached, null);

        builder.addDoubleProperty("SYSID_Voltage", ()->pivot_motor.getBusVoltage() * pivot_motor.getAppliedOutput(), null);
        builder.addDoubleProperty("SYSID_Velocity", ()->abs_encoder.getVelocity()  * 2 * Math.PI, null); // radians/min
        builder.addDoubleProperty("SYSID_Position", ()->abs_encoder.getPosition() * 2 * Math.PI, null); // radians
    }

    /**
     * Creates a command to move to a specified reference position.
     * 
     * @param reference the desired position
     * @return the command
     */
    public Command go_to_reference(double reference) {
        return startRun(
            () -> {
                this.reference = adjusted_reference(reference); // Adjust the reference position to ensure it's within bounds
                timer.reset(); // Reset the timer for motion profiling
                start_state = new TrapezoidProfile.State(abs_encoder.getPosition(), abs_encoder.getVelocity()); // Set the starting state
                goal_state = new TrapezoidProfile.State(this.reference, 0); // Set the goal state (target position)
            },
            () -> {
                current_state = profile.calculate(timer.get(), start_state, goal_state); // Calculate the next motion state
                command_output(current_state.position, current_state.velocity); // Output the command to the motor
            })
            .until(() -> profile.isFinished(timer.get())) // Stop when the profile is finished
            .withName("Go To Reference"); // Name the command
    }

    /**
     * Stops the motor immediately.
     * 
     * @return the stop command
     */
    public Command stop_motor() {
        return runOnce(() -> pivot_motor.stopMotor()); // Immediately stop the motor
    }

    /**
     * Holds the arm at its current position.
     * 
     * @return the hold position command
     */
    public Command hold_position() {
        return run(() -> command_output()) // Hold position by continuing to output the current command
            .withName("Hold Position"); // Name the command
    }

    /**
     * Outputs the calculated command to the motor controller.
     * This is used to control the arm's position and velocity.
     */
    private void command_output() {
        command_output(reference, 0); // Use the reference position and zero velocity
    }

    /**
     * Outputs the calculated command to the motor controller based on position and velocity.
     * 
     * @param position the target position
     * @param velocity the target velocity
     */
    private void command_output(double position, double velocity) {
        // Calculate feedforward term based on current position and velocity
        FF = feedforward.calculate(abs_encoder.getPosition() * 2 * Math.PI, velocity);
        // Set the motor reference for position control
        pivot_controller.setReference(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, FF, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    /**
     * Adjusts the reference position to ensure it stays within the limits.
     * 
     * @param reference the desired reference
     * @return the adjusted reference
     */
    private double adjusted_reference(double reference) {
        // Ensure the reference is within a safe range
        return Math.max(0.1, Math.min(reference, 0.35));
    }

    /**
     * Updates the PID controller with the current gain values.
     */
    private void update_controller_PID() {
        pivot_config.closedLoop
            .pid(P, I, D) // Update PID gains
            .iZone(IZone) // Update integral zone
            .iMaxAccum(IMaxAccum); // Update max accumulation for integral term
        // Reapply the configuration to the motor controller
        pivot_motor.configure(pivot_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Checks if the reference position has been updated significantly.
     * 
     * @return true if the reference has changed significantly, false otherwise
     */
    public boolean is_reference_updated() {
        // If the reference has changed beyond a small epsilon threshold
        if (Math.abs(reference - previous_reference) > EPSILON) {
            previous_reference = reference; // Update the previous reference
            return true; // Return true to indicate the reference was updated
        }
        return false; // No significant change
    }

    /**
     * Checks if the forward soft limit has been reached.
     * 
     * @return true if the forward limit is reached, false otherwise
     */
    private boolean isForwardLimitReached() {
        return abs_encoder.getPosition() > 0.4 && pivot_motor.getAppliedOutput() > 0; // Check if the position exceeds the forward limit
    }

    /**
     * Checks if the reverse soft limit has been reached.
     * 
     * @return true if the reverse limit is reached, false otherwise
     */
    private boolean isReverseLimitReached() {
        return abs_encoder.getPosition() < 0.05 && pivot_motor.getAppliedOutput() < 0; // Check if the position is below the reverse limit
    }

    /**
     * Checks if any soft limit (forward or reverse) has been reached.
     * 
     * @return true if either limit is reached, false otherwise
     */
    private boolean isSoftLimitReached(){
        return isForwardLimitReached() || isReverseLimitReached(); // Check if either limit is reached
    }
}
