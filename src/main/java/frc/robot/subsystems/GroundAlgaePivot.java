package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.GroundAlgaePivotConstants.*;

public class GroundAlgaePivot extends SubsystemBase {

    // Spark Max, Encoder, Controller
    private final SparkMax pivotMotor = new SparkMax(kMotorId,
            MotorType.kBrushless);
    private final AbsoluteEncoder pivotMotorEncoder = pivotMotor.getAbsoluteEncoder();
    private final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController pivotMotorController = pivotMotor.getClosedLoopController();

    // Arm Feedforward
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(kFeedforwardKs,
            kFeedforwardKg, kFeedforwardKv,
            kFeedforwardKa, kFeedforwardDtSeconds);

    // Trapezoid Motion Profile
    private final TrapezoidProfile pivotTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            kMaxVelocity, kMaxAcceleration));
    private TrapezoidProfile.State pivotTrapezoidStart = new TrapezoidProfile.State();
    private TrapezoidProfile.State pivotTrapezoidCurrent = new TrapezoidProfile.State();
    private TrapezoidProfile.State pivotTrapezoidGoal = new TrapezoidProfile.State();

    // Timer
    private final Timer timer = new Timer();

    // Setpoint
    // Will need a better solution if the arm will not be at the same position every
    // time
    private double pivotSetpoint = kSetpointStow;

    public GroundAlgaePivot() {

        // Spark configuration
        pivotMotorConfig
                .inverted(kMotorInverted)
                .idleMode(kMotorIdleMode)
                .smartCurrentLimit(kCurrentLimit);

        pivotMotorConfig.encoder
                .positionConversionFactor(kMotorPositionConversionFactor)
                .velocityConversionFactor(kMotorVelocityConversionFactor);

        pivotMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(kP, kI, kD)
                .outputRange(-1, 1);

        pivotMotorConfig.absoluteEncoder
                .zeroOffset(kMotorEncoderOffset);

        pivotMotorConfig.softLimit
                .forwardSoftLimitEnabled(kForwardSoftLimitEnabled)
                .forwardSoftLimit(kForwardSoftLimit)
                .reverseSoftLimitEnabled(kReverseSoftLimitEnabled)
                .reverseSoftLimit(kReverseSoftLimit);

        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    // Set pivot output with feedforward using position, velocity
    private void setPivotOutput(double position, double velocity) {
        pivotMotorController.setReference(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0,
                pivotFeedforward.calculate(position, velocity), SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    // Command to pivot to a setpoint
    public Command pivotToSetpoint(double newSetpoint, String setpointName) {

        return startRun(
                () -> {
                    pivotSetpoint = newSetpoint;
                    timer.reset();
                    pivotTrapezoidStart = new TrapezoidProfile.State(pivotMotorEncoder.getPosition(),
                            pivotMotorEncoder.getVelocity());
                    pivotTrapezoidGoal = new TrapezoidProfile.State(this.pivotSetpoint, 0);
                },
                () -> {
                    pivotTrapezoidCurrent = pivotTrapezoidProfile.calculate(timer.get(), pivotTrapezoidStart,
                            pivotTrapezoidGoal);
                    setPivotOutput(pivotTrapezoidCurrent.position, pivotTrapezoidCurrent.velocity);
                }).until(() -> pivotTrapezoidProfile.isFinished(timer.get())).withName("Go to " + setpointName);

    }

    //Holding state
    public Command holdState() {
        return run(() -> {
            setPivotOutput(pivotSetpoint, 0);
        });
    }


    // Commands to pivot to some specific setpoints
    public Command goToStow() {
        return pivotToSetpoint(kSetpointStow, "Stow");
    }

    public Command goToIntake() {
        return pivotToSetpoint(kSetpointIntake, "Intake");
    }    

    public Command goToScore() {
        return pivotToSetpoint(kSetpointScore, "Score");
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

}
