package frc.robot.subsystems;
import static frc.robot.Constants.PincerConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pincer extends SubsystemBase{

    
    private final SparkMax pincerMotor = new SparkMax(kPincerMotorId, MotorType.kBrushless);
    private final AbsoluteEncoder pincerAbsEncoder;
    private final SparkMaxConfig pincerConfig  = new SparkMaxConfig();
    private final SparkClosedLoopController pincerController;

    private final Debouncer algaeDebouncer = new Debouncer(0.1);
    private final Debouncer algaeDebouncer1 = new Debouncer(0.1);
    private final Debouncer algaeDebouncer2 = new Debouncer(0.1);


    private boolean pincerPIDUpdated = true;

    private final SparkMax intakeMotor = new SparkMax(kIntakeMotorId, MotorType.kBrushless);
    private final SparkMaxConfig intakeConfig  = new SparkMaxConfig();

    private final LaserCan laserCan = new LaserCan(kLaserCanId);
    private final Debouncer laserCanDebouncer = new Debouncer(0.1);

    public double pincerSetpoint = kFunnelPosition;
    private double lastLaserCANReading = 100;

    private boolean clampingOnAlgae = false;

    // Use current sensing for the algae

    public Pincer(){
        // Grab objects from spark maxs
        pincerAbsEncoder = pincerMotor.getAbsoluteEncoder();
        pincerController = pincerMotor.getClosedLoopController();

        pincerConfig
            .inverted(kPincerMotorInverted)
            .idleMode(kPincerMotorIdleMode)
            .smartCurrentLimit(kPincerMotorSmartCurrentLimit);
        pincerConfig
            .encoder.positionConversionFactor(kPositionConversionFactor);
        pincerConfig
            .closedLoop
                .feedbackSensor(kPincerMotorFeedbackSensor)
                .pid(kPincerP, kPincerI, kPincerD)
                .outputRange(kPincerMotorMinOutput, kPincerMotorMaxOutput);
        pincerConfig
            .absoluteEncoder
                .zeroOffset(kPincerMotorAbsoluteEncoderOffset)
                .zeroCentered(kPincerMotorZeroCentered);
        pincerConfig
            .softLimit
                .reverseSoftLimitEnabled(kSoftLimitsEnabled)
                .forwardSoftLimitEnabled(kSoftLimitsEnabled)
                .forwardSoftLimit(kPincerMaxAngle)
                .reverseSoftLimit(kPincerMinAngle);  

        intakeConfig
            .inverted(kIntakeMotorInverted)
            .idleMode(kIntakeMotorIdleMode)
            .smartCurrentLimit(kIntakeMotorSmartCurrentLimit);


        pincerMotor.configure(pincerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //Constants.kMotorBurnDelay();
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //Constants.kMotorBurnDelay();
        //pincerSetpoint = kStowPosition;

        pincerController.setReference(pincerSetpoint, SparkMax.ControlType.kPosition);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        // Absolute Encoder
        builder.addDoubleProperty("Pincer Absolute Position", () -> pincerAbsEncoder.getPosition(), null);
        builder.addDoubleProperty("Pincer Absolute Velocity", () -> pincerAbsEncoder.getVelocity(), null);
        // Pincer Motor
        builder.addDoubleProperty("Pincer Motor Output", () -> pincerMotor.getAppliedOutput(), null);
        builder.addDoubleProperty("Pincer Motor Temperature", () -> pincerMotor.getMotorTemperature(), null);
        builder.addDoubleProperty("Pincer Motor Output Current", () -> pincerMotor.getOutputCurrent(), null);
        // Intake Motor
        builder.addDoubleProperty("Intake Motor Output", () -> intakeMotor.getAppliedOutput(), null);
        builder.addDoubleProperty("Intake Motor Temperature", () -> intakeMotor.getMotorTemperature(), null);
        builder.addDoubleProperty("Intake Motor Output Current", () -> intakeMotor.getOutputCurrent(), null);

        builder.addDoubleProperty("Pincer Setpoint", () -> pincerSetpoint, null);

        builder.addDoubleProperty("Pincer .get()", () -> pincerMotor.get(), null);
        builder.addDoubleProperty("Intake .get()", () -> intakeMotor.get(), null);

        // PID Tuning
        builder.addDoubleProperty("Pincer kP", () -> kPincerP, value -> { kPincerP = value; pincerPIDUpdated = true;});
        builder.addDoubleProperty("Pincer kI", () -> kPincerI, value -> { kPincerI = value; pincerPIDUpdated = true;});
        builder.addDoubleProperty("Pincer kD", () -> kPincerD, value -> { kPincerD = value; pincerPIDUpdated = true;});

        builder.addBooleanProperty("Has Coral",() -> hasCoral(),null);
        builder.addBooleanProperty("Has Algae",() -> hasAlgae(),null);
        
        builder.addBooleanProperty("Has Algae (Velocity Condition)", () -> algaeDebouncer1.calculate(Math.abs(pincerAbsEncoder.getVelocity()) < 0.1), null);
        builder.addBooleanProperty("Has Algae (Position Condition)", () -> algaeDebouncer.calculate(pincerAbsEncoder.getPosition() < -0.08), null);
    
        builder.addBooleanProperty("Clamping on Algae", () -> clampingOnAlgae, null);
    }

    @Override
    public void periodic(){

        if (pincerPIDUpdated){
            pincerConfig
                .closedLoop.pid(kPincerP, kPincerI, kPincerD);
            pincerMotor.configure(pincerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            pincerPIDUpdated = false;
        }

        SmartDashboard.putData(this);
    }

    public double getPincerSetpoint(){
        return pincerSetpoint;
    }

    /**@return True if the current draw on the intake motor is over the algae threshold
     */
    public boolean hasAlgae(){
        return algaeDebouncer.calculate(Math.abs(pincerAbsEncoder.getVelocity()) < 0.1 && pincerAbsEncoder.getPosition() < -0.08);
    }

    public boolean hasCoral(){
        Measurement measurement = laserCan.getMeasurement();
        double distance_mm = 100;

        if (measurement != null) { // Ensure the measurement is not null
            distance_mm = measurement.distance_mm;
            lastLaserCANReading = distance_mm;
            // Do something with distance_mm
        }
        else{
            distance_mm = lastLaserCANReading;
        }
        return laserCanDebouncer.calculate( distance_mm < 25);
    }

    /**@return True if the intake photogate is tripped
     */
    /*
    public boolean hasCoral(){
        return intakePhotogate.get();
    } */
    // public boolean hasCoral(){
    //     return intakePhotogate.get();
    // }

    public void setPincerOutput(double setpoint) {
        pincerController.setReference(setpoint, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);        
    } 

    /** Moves the pincer to the specified setpoint
     * @param setpoint The desired position for the pincer
     * @return Command
     */
    private Command pincerToSetpoint(double setpoint) {
        return runOnce(() -> {pincerSetpoint = setpoint; clampingOnAlgae = false; });
    }

    /** Moves the pincer to the algae-grabbing position
     * @return Command
     */
    public Command pincerAlgae() {
        return pincerToSetpoint(kAlgaePosition);
    }

    public Command reefAlgae(){
        return pincerToSetpoint(Constants.PincerConstants.setPoints.get(
            Constants.robotStates.pincerStates.ALGAEINTAKE
        ));
    }

    /** Moves the pincer to the stowed position
     * @return Command
     */
    public Command stowPincer() {
        return pincerToSetpoint(kStowPosition);
    }

    /** Moves the pincer to the funneling position
     * @return Command
     */
    public Command pincerFunnel() {
        return pincerToSetpoint(kFunnelPosition);
    }

    public Command pincerFunnel2() {
        return pincerToSetpoint(kFunnel2Position);
    }

    public Command algaeScore(){
        return pincerToSetpoint(Constants.PincerConstants.setPoints.get(
            Constants.robotStates.pincerStates.ALGAESCORE
        ));
    }

    public void holdIntake() {
        if(hasAlgae()) {
            intakeMotor.stopMotor();
        }else{
            intakeMotor.set(-0.25);
            //intakeMotor.stopMotor();
        } 
    }

    

    public void holdPincer() {
        clampingOnAlgae = true;
        pincerMotor.set(-0.25);
    }
    
    public Command pincerAlgaeHold() {
        return startEnd(() -> {holdPincer(); holdIntake();}, 
                        () -> {
                            if (!hasAlgae()){
                                setPincerOutput(pincerSetpoint);
                            }                  
                        }).until(() -> pincerAbsEncoder.getPosition() < -0.250 || hasAlgae());
    }

    /** Runs the intake motor at the intake speed
     * @return Command
     */
    public Command intake() {
        return runOnce(() -> intakeMotor.set(kIntakeSpeed));
    }

    /** Runs the intake motor at the exhaust speed
     * @return Command
     */
    public Command exhaust() {
        return runOnce(() -> intakeMotor.set(kExhaustSpeed));
    }

    public Command manualIntake() {
        return runEnd(() -> intakeMotor.set(kIntakeSpeed), () -> intakeMotor.stopMotor());
    } 

    public Command manualExhaust() {
        return runEnd(() -> intakeMotor.set(kExhaustSpeed), () -> intakeMotor.stopMotor());
    } 
    

    /** Stops the intake motor
     * @return Command
     */
    public Command stopIntake() {
        return runOnce(() -> intakeMotor.stopMotor());
    }
    
    public Command holdState(){
        return run(() -> {
            if (!clampingOnAlgae){
                setPincerOutput(pincerSetpoint);
            }else{
                pincerMotor.set(-0.25);
                if(pincerAbsEncoder.getPosition() < -0.250 /*|| !hasAlgae()*/) clampingOnAlgae = false;
            }
        }).withName("Hold State");

        //return Commands.idle(this);
    }

}
