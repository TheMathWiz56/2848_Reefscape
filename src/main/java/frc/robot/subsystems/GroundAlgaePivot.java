package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundAlgaePivotConstants;
import frc.robot.Constants.GroundAlgaePivotConstants;

public class GroundAlgaePivot extends SubsystemBase {

    // Spark Max and Encoder
    private final SparkMax pivotMotor = new SparkMax(GroundAlgaePivotConstants.kMotorId,
            MotorType.kBrushless);
    private final AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
    private final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

    // Arm Feedforward
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(GroundAlgaePivotConstants.kFeedforwardKs,
            GroundAlgaePivotConstants.kFeedforwardKg, GroundAlgaePivotConstants.kFeedforwardKv,
            GroundAlgaePivotConstants.kFeedforwardKa, GroundAlgaePivotConstants.kFeedforwardDtSeconds);

    // Trapezoid Motion Profile
    private final TrapezoidProfile pivotTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            GroundAlgaePivotConstants.kMaxVelocity, GroundAlgaePivotConstants.kMaxAcceleration));
    private TrapezoidProfile.State pivotTrapezoidStart = new TrapezoidProfile.State();
    private TrapezoidProfile.State pivotTrapezoidCurrent = new TrapezoidProfile.State();
    private TrapezoidProfile.State pivotTrapezoidGoal = new TrapezoidProfile.State();

    public GroundAlgaePivot() {

        // Spark configuration
        pivotMotorConfig
                .inverted(GroundAlgaePivotConstants.kMotorInverted)
                .idleMode(GroundAlgaePivotConstants.kMotorIdleMode)
                .smartCurrentLimit(GroundAlgaePivotConstants.kCurrentLimit);

        pivotMotorConfig.encoder
                .positionConversionFactor(0.0)
                .velocityConversionFactor(0.0);

        pivotMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(GroundAlgaePivotConstants.kP, GroundAlgaePivotConstants.kI, GroundAlgaePivotConstants.kD)
                .outputRange(-1, 1);

        pivotMotorConfig.absoluteEncoder
                .zeroOffset(GroundAlgaePivotConstants.kMotorEncoderOffset);

        pivotMotorConfig.softLimit
                .reverseSoftLimitEnabled(false)
                .forwardSoftLimitEnabled(false)
                .forwardSoftLimit(0.0)
                .reverseSoftLimit(0.0);

        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

}
