package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundAlgaePivotConstants;

public class GroundAlgaePivot extends SubsystemBase {

    private final SparkMax pivotMotor = new SparkMax(GroundAlgaePivotConstants.kMotorId,
            MotorType.kBrushless);
    private final AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
    private final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

    public GroundAlgaePivot() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

}
