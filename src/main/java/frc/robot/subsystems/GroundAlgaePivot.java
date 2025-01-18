package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundAlgaePivot extends SubsystemBase {

    private final SparkMax groundAlgaePivotMotor = new SparkMax(0, MotorType.kBrushless);

}
