package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.PincerConstants;

import static frc.robot.Constants.ArmConstants.kArmPhotogateId;
import static frc.robot.Constants.PincerConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pincer extends SubsystemBase{

    private final SparkMax pincerMotor = new SparkMax(kPincerMotorId, MotorType.kBrushless);
    private final SparkMaxConfig pincerConfig  = new SparkMaxConfig();
    private final SparkClosedLoopController pincerController;

    private final SparkMax armIntakeMotor = new SparkMax(kIntakeMotorId, MotorType.kBrushless);
    private final SparkMaxConfig armIntakeConfig  = new SparkMaxConfig();

    // Photogate (beam break)
    private final DigitalInput armPhotogate = new DigitalInput(kArmPhotogateId);

    public Pincer(){
        // Grab objects from spark maxs
        pincerController = pincerMotor.getClosedLoopController();

        pincerConfig
            .inverted(PincerConstants.kPincerMotorInverted)
            .idleMode(PincerConstants.kPincerMotorIdleMode)
            .smartCurrentLimit(PincerConstants.kPincerMotorSmartCurrentLimit);
        pincerConfig
            .closedLoop
                .feedbackSensor(PincerConstants.kPincerMotorFeedbackSensor)
                .pid(PincerConstants.kPincerP, PincerConstants.kPincerI, PincerConstants.kPincerD)
                .outputRange(PincerConstants.kPincerMotorMinOutput, PincerConstants.kPincerMotorMaxOutput);
        pincerConfig
            .absoluteEncoder
                .zeroOffset(PincerConstants.kPincerMotorAbsoluteEncoderOffset);

        armIntakeConfig
            .inverted(kIntakeMotorInverted)
            .idleMode(kIntakeMotorIdleMode)
            .smartCurrentLimit(kIntakeMotorSmartCurrentLimit);
        armIntakeConfig
            .closedLoop
                .pid(kIntakeP, kIntakeI, kIntakeD)
                .outputRange(kIntakeMotorMinOutput, kIntakeMotorMaxOutput);


        pincerMotor.configure(pincerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Constants.kMotorBurnDelay();
        armIntakeMotor.configure(armIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Constants.kMotorBurnDelay();

    }
    
}
