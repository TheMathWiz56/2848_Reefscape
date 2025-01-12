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
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Arm extends SubsystemBase{

    private final SparkMax pivot_motor;
    private final SparkMaxConfig pivot_config;
    private final SparkClosedLoopController pivot_controller;
    private final AbsoluteEncoder abs_encoder;

    private final ArmFeedforward feedforward;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State goal_state, start_state, current_state;

    // Reference means the same thing as setpoint
    private double reference, previous_reference, P, I, D, FF, IZone, IMaxAccum;

    private boolean PIDupdated;

    // Timer for stepping between motion profile setpoints
    private final Timer timer = new Timer();

    private final SysIdRoutine routine;

    //  _____________________________________________________________________________________________________________
    // Define a small tolerance for floating-point comparison. 
    // (JOSEPH) SHOULD BE PLACED IN CONSTANTS FILE ONCE MERGED, also should place the double comparison in utils ...
    private static final double EPSILON = 1e-9;
    private static boolean hasSignificantChange(double current, double previous) {
        return Math.abs(current - previous) > EPSILON;
    }
    // _____________________________________________________________________________________________________________

    public Arm (){
        pivot_motor = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
        pivot_config = new SparkMaxConfig();
        abs_encoder = pivot_motor.getAbsoluteEncoder();
        pivot_controller = pivot_motor.getClosedLoopController();

        reference = .25;
        previous_reference = .25;
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

        // 19 deg -(0.0527777) + 0.1745584
        pivot_config.absoluteEncoder
            .zeroOffset(0.12107807);

        // Units of the gain values will dictate units of the computed feedforward.
        feedforward = new ArmFeedforward(0, 0.035173, 1.1233, 0.11294); //0.02 kG, 0.75kV . UNITS: Volts???
        // 0'd ks because of slop, was 0.32415

        // Rev/s and Rev/s/s         |        1.33 rev/s and ~1.33 rev/s/s MAX
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5,1));

        // Creates a SysIdRoutine
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::voltageDrive, null, this));

        // Start at subsystem initialization
        timer.start();

        // Burn configuration to the spark max in case of power loss
        pivot_motor.configure(pivot_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Timer.delay(.1); // give time for the controller to properly configure itself
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction)
                    .until( () -> (abs_encoder.getPosition() > .4 && pivot_motor.getAppliedOutput() > 0) || (abs_encoder.getPosition() < .05 && pivot_motor.getAppliedOutput() < 0))
                    .andThen(stop_motor()); // motion limits
    }
      
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction)
                    .until( () -> (abs_encoder.getPosition() > .4 && pivot_motor.getAppliedOutput() > 0) || (abs_encoder.getPosition() < .05 && pivot_motor.getAppliedOutput() < 0))
                    .andThen(stop_motor()); // motion limits
    }

    public void voltageDrive(Voltage voltage){
        SmartDashboard.putNumber("SysID magnitude", voltage.magnitude()); // works
        pivot_motor.setVoltage(voltage.magnitude()); // .magnitude returns the signed double value
        // test to see what magnitude returns first
    }

    @Override
    public void periodic() {
        if (PIDupdated){
            update_controller_PID();
            PIDupdated = false;
        }

        SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void initSendable(SendableBuilder builder) {

        // This might be necessary to have the sendable builder work
        super.initSendable(builder);
        
        builder.addDoubleProperty("previous reference", ()->previous_reference, null);
        builder.addDoubleProperty("reference", ()->reference, value->reference = adjusted_reference(value));
        // Might also be able to add the sendable builder for the encoder to this: check later
        builder.addDoubleProperty("position", ()->abs_encoder.getPosition(), null);

        builder.addDoubleProperty("P", () -> P, value -> { P = value; PIDupdated = true; });
        builder.addDoubleProperty("I", () -> I, value -> { I = value; PIDupdated = true; });
        builder.addDoubleProperty("IZone", () -> IZone, value -> { IZone = value; PIDupdated = true; });
        builder.addDoubleProperty("IMaxAccum", () -> IMaxAccum, value -> { IMaxAccum = value; PIDupdated = true; });
        builder.addDoubleProperty("D", () -> D, value -> { D = value; PIDupdated = true; });

        builder.addDoubleProperty("timer", ()-> timer.get(), null);
        builder.addDoubleProperty("FF", ()->FF, null);
        builder.addDoubleProperty("MP_Position", ()->goal_state.position, null);
        builder.addDoubleProperty("MP_Velocity", ()->goal_state.velocity, null);
        builder.addDoubleProperty("motor output", ()->pivot_motor.getAppliedOutput(), null);
        builder.addDoubleProperty("Error", ()->goal_state.position - abs_encoder.getPosition(), null);
        builder.addDoubleProperty("Error_Degrees", ()-> (goal_state.position - abs_encoder.getPosition())*360, null);

        builder.addDoubleProperty("SYSID_Voltage", ()->pivot_motor.getBusVoltage() * pivot_motor.getAppliedOutput(), null);
        builder.addDoubleProperty("SYSID_Velocity", ()->abs_encoder.getVelocity()  * 2 * Math.PI, null); // radians/min
        builder.addDoubleProperty("SYSID_Position", ()->abs_encoder.getPosition() * 2 * Math.PI, null); // radians
        builder.addBooleanProperty("Reverse Soft Limit", () -> abs_encoder.getPosition() < .05 && pivot_motor.getAppliedOutput() < 0, null);
        builder.addBooleanProperty("Forward Soft Limit", () -> abs_encoder.getPosition() > .4 && pivot_motor.getAppliedOutput() > 0, null);
        builder.addBooleanProperty("Combined Limits", ()-> (abs_encoder.getPosition() > .4 && pivot_motor.getAppliedOutput() > 0) || (abs_encoder.getPosition() < .05 && pivot_motor.getAppliedOutput() < 0), null);
    }

    
    public Command go_to_reference(double reference){
        return startRun(
                ()-> {
                    // Adjust Reference for limits
                    this.reference = adjusted_reference(reference);
                    // Reset the timer for the motion profile
                    timer.reset();
                    // update the start and goal states
                    start_state = new TrapezoidProfile.State(abs_encoder.getPosition(), abs_encoder.getVelocity());
                    goal_state = new TrapezoidProfile.State(this.reference, 0);
                }, 
                ()->{
                    // Calculates the position and velocity for the profile at a time t where the start state is at time t = 0.
                    current_state = profile.calculate(timer.get(), start_state, new TrapezoidProfile.State(this.reference, 0));

                    command_output(current_state.position, current_state.velocity);
                })
                .until(()->profile.isFinished(timer.get()))
                .withName("Go To Reference");
    }

    public Command go_to_reference(){
        return startRun(
                ()-> {
                    // Reset the timer for the motion profile
                    timer.reset();
                    // record the initial state
                    start_state = new TrapezoidProfile.State(abs_encoder.getPosition(), abs_encoder.getVelocity());
                    goal_state = new TrapezoidProfile.State(reference, 0);
                }, 
                ()->{
                    // Calculates the position and velocity for the profile at a time t where the start state is at time t = 0.
                    current_state = profile.calculate(timer.get(), start_state, new TrapezoidProfile.State(reference, 0));

                    command_output(current_state.position, current_state.velocity);
                })
                .until(()->profile.isFinished(timer.get()))
                .withName("Go To Reference");
    }

    public Command stop_motor(){
        return runOnce(()->pivot_motor.stopMotor());
    }

    public Command hold_position(){
        return run(()->command_output())
                .withName("Hold Position");
    }

    private void command_output(){
        command_output(reference, 0);
    }

    private void command_output(double position, double velocity){
        // Calculates the feedforward using the position for the kG and velocity setpoint for kV. 
        // MIGHT need to divide by the battery voltage, don't need to divide by battery voltage because initial feedforward gains are in percent output
        // import edu.wpi.first.units. should allow you to specifiy units for numbers?
        FF = feedforward.calculate(abs_encoder.getPosition() * 2 * Math.PI, current_state.velocity);
        // convert to radians, could use a position conversion factor in abs setup instead

        // Feed the PID the current position setpoint from the motion profile with the feedforward component (percentoutput)
        pivot_controller.setReference(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, FF, SparkClosedLoopController.ArbFFUnits.kPercentOut); // should be a command to keep current position
        // the internal pid controller is using voltage control, so the gains correspond to a voltage increase. ~ max around 12, depends on battery
    }


    private double adjusted_reference(double reference){
        reference = Math.min(reference, 0.35);
        reference = Math.max(reference, 0.1);
        return reference;
    }

    /**
     * Updates the PID controller with the current values of P, I, and D.
     */
    private void update_controller_PID() {
        // Set the closed loops gains of the spark max controller
        pivot_config.closedLoop
            .pid(P, I, D)
            .iZone(IZone)
            .iMaxAccum(IMaxAccum); 
        pivot_motor.configure(pivot_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /*
     * Checks if the current setpoint has significantly changed from the previous setpoint.
     *
     * @return true if the setpoint has changed, false otherwise.
     */
    public boolean is_reference_updated() {
        // Check if the setpoint has changed significantly
        if (hasSignificantChange(reference, previous_reference)) {
            previous_reference = reference;
            return true; // A significant update has occurred
        }
        return false; // No significant change
    }

    
}