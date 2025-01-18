package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundAlgaePivotConstants;

public class GroundAlgaePivot extends SubsystemBase {

    // Spark Max and Encoder
    private final SparkMax pivotMotor = new SparkMax(GroundAlgaePivotConstants.kMotorId,
            MotorType.kBrushless);
    private final AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
    private final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

    // Arm Feedforward
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(0, 0, 0, 0, 0);

    // Trapezoid Motion Profile
    private final TrapezoidProfile pivotTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0, 0));
    private final TrapezoidProfile.State pivotTrapezoidInitial = new TrapezoidProfile.State();
    private final TrapezoidProfile.State pivotTrapezoidFinal = new TrapezoidProfile.State();

    public GroundAlgaePivot() {
        

        
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

}
