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
    private final SparkMax pivot_motor;
    private final SparkMaxConfig pivot_config;
    private final SparkClosedLoopController pivot_controller;
    private final AbsoluteEncoder abs_encoder;

    // Feedforward controller for arm motion
    private final ArmFeedforward feedforward;

    // Motion profiling and state tracking
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State goal_state, start_state, current_state;

    // PID and feedforward gains
    private double reference, previous_reference, P, I, D, FF, IZone, IMaxAccum;
    private boolean PIDupdated;

    // Timer for motion profile timing
    private final Timer timer = new Timer();

    // System identification routine
    private final SysIdRoutine routine;

    // Tolerance for floating-point comparison
    private static final double EPSILON = 1e-9;

    /**
     * Constructor for the Arm subsystem. Initializes the motor, encoders, feedforward, 
     * and other configuration components.
     */
    public Arm() {
        pivot_motor = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
        pivot_config = new SparkMaxConfig();
        abs_encoder = pivot_motor.getAbsoluteEncoder();
        pivot_controller = pivot_motor.getClosedLoopController();

        // Initial setpoints and PID gains
        reference = 0.25;
        previous_reference = 0.25;
        goal_state = new TrapezoidProfile.State();
        start_state = new TrapezoidProfile.State();
        current_state = new TrapezoidProfile.State();
        P = 7;
        I = 0;
        IZone = 0;
        IMaxAccum = 0;
        D = 1.233;
        FF = 0;
        PIDupdated = false;

        // Configure the pivot motor
        pivot_config
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);
        pivot_config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(P, I, D)
            .iZone(IZone)
            .iMaxAccum(IMaxAccum)
            .outputRange(-1, 1);

        // Absolute encoder configuration
        pivot_config.absoluteEncoder
            .zeroOffset(0.12107807);

        // Arm feedforward configuration (units in volts)
        feedforward = new ArmFeedforward(0, 0.035173, 1.1233, 0.11294);

        // Motion profile constraints (units: rev/s and rev/s^2)
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5, 1));

        // Initialize system identification routine
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::voltageDrive, null, this));

        // Start timer
        timer.start();

        // Burn configurations to motor controller
        pivot_motor.configure(pivot_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Timer.delay(0.1); // Allow configuration to apply
    }

    /**
     * Creates a command for system identification in quasistatic mode.
     * 
     * @param direction the direction of motion
     * @return the command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction)
            .until(() -> isSoftLimitReached())
            .andThen(stop_motor());
    }

    /**
     * Creates a command for system identification in dynamic mode.
     * 
     * @param direction the direction of motion
     * @return the command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction)
            .until(() -> isSoftLimitReached())
            .andThen(stop_motor());
    }

    /**
     * Drives the motor using a voltage input.
     * 
     * @param voltage the voltage to apply
     */
    public void voltageDrive(Voltage voltage) {
        SmartDashboard.putNumber("SysID magnitude", voltage.magnitude());
        pivot_motor.setVoltage(voltage.magnitude());
    }

    @Override
    public void periodic() {
        if (PIDupdated) {
            update_controller_PID();
            PIDupdated = false;
        }

        SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {
        // Called during simulation mode
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

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
                this.reference = adjusted_reference(reference);
                timer.reset();
                start_state = new TrapezoidProfile.State(abs_encoder.getPosition(), abs_encoder.getVelocity());
                goal_state = new TrapezoidProfile.State(this.reference, 0);
            },
            () -> {
                current_state = profile.calculate(timer.get(), start_state, goal_state);
                command_output(current_state.position, current_state.velocity);
            })
            .until(() -> profile.isFinished(timer.get()))
            .withName("Go To Reference");
    }

    /**
     * Stops the motor immediately.
     * 
     * @return the stop command
     */
    public Command stop_motor() {
        return runOnce(() -> pivot_motor.stopMotor());
    }

    /**
     * Holds the arm at its current position.
     * 
     * @return the hold position command
     */
    public Command hold_position() {
        return run(() -> command_output())
            .withName("Hold Position");
    }

    /**
     * Outputs the calculated command to the motor controller.
     */
    private void command_output() {
        command_output(reference, 0);
    }

    /**
     * Outputs the calculated command to the motor controller based on position and velocity.
     * 
     * @param position the target position
     * @param velocity the target velocity
     */
    private void command_output(double position, double velocity) {
        FF = feedforward.calculate(abs_encoder.getPosition() * 2 * Math.PI, velocity);
        pivot_controller.setReference(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, FF, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    /**
     * Adjusts the reference position to ensure it stays within the limits.
     * 
     * @param reference the desired reference
     * @return the adjusted reference
     */
    private double adjusted_reference(double reference) {
        return Math.max(0.1, Math.min(reference, 0.35));
    }

    /**
     * Updates the PID controller with the current gain values.
     */
    private void update_controller_PID() {
        pivot_config.closedLoop
            .pid(P, I, D)
            .iZone(IZone)
            .iMaxAccum(IMaxAccum);
        pivot_motor.configure(pivot_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Checks if the reference position has been updated significantly.
     * 
     * @return true if the reference has changed significantly, false otherwise
     */
    public boolean is_reference_updated() {
        if (Math.abs(reference - previous_reference) > EPSILON) {
            previous_reference = reference;
            return true;
        }
        return false;
    }

    /**
     * Checks if the forward soft limit has been reached.
     * 
     * @return true if the forward limit is reached, false otherwise
     */
    private boolean isForwardLimitReached() {
        return abs_encoder.getPosition() > 0.4 && pivot_motor.getAppliedOutput() > 0;
    }

    /**
     * Checks if the reverse soft limit has been reached.
     * 
     * @return true if the reverse limit is reached, false otherwise
     */
    private boolean isReverseLimitReached() {
        return abs_encoder.getPosition() < 0.05 && pivot_motor.getAppliedOutput() < 0;
    }

    private boolean isSoftLimitReached(){
        return isForwardLimitReached() && isReverseLimitReached();
    }
}
