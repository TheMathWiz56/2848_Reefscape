package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {

    /**
     * Delays the program to allow the motor controller to burn the configure to memory
     */
    public static void kMotorBurnDelay(){
        Timer.delay(0.1);
    }

    /**
     * Adjusts the input reference to ensure it stays within the specified range.
     *
     * @param input the desired reference value
     * @param min the minimum allowable value
     * @param max the maximum allowable value
     * @return the adjusted reference value, clamped between min and max
     */
    public static double kClamp(double input, double min, double max) {
        return Math.max(min, Math.min(input, max));
    }

    // Constants relating to the drivetrain are probably in TunerConstants

    public static class ControllerConstants {
        public static final int kDriverController = 0;
        // One of the operator controllers is going to be for emergencies if all else
        // fails
        public static final int kOperatorController1 = 1;
        public static final int kOperatorController2 = 2;
    }

    public static class ArmConstants {
        // ID's
        public static final int kPivotMotorId = 0;
    
        // Pivot Motor Controller Settings
        public static final boolean kPivotMotorInverted = false;
        public static final IdleMode kPivotMotorIdleMode = IdleMode.kCoast;
        public static final int kPivotMotorSmartCurrentLimit = 40;
    
        // Pivot PID/Feedforward Settings
        public static double kPivotP = 0.0;
        public static double kPivotI = 0.0;
        public static double kPivotD = 0.0;
        public static final FeedbackSensor kPivotMotorFeedbackSensor = FeedbackSensor.kAbsoluteEncoder;
        public static final double kPivotMotorMinOutput = -1;
        public static final double kPivotMotorMaxOutput = 1;
        public static final double kPivotMotorAbsoluteEncoderOffset = 0.0;
        public static double kPivotMotorks = 0.0;
        public static double kPivotMotorkg = 0.0;
        public static double kPivotMotorkv = 0.0;
        public static double kPivotMotorka = 0.0;
        public static double kPivotMotorMaxVelocity = 0.0;
        public static double kPivotMotorMaxAcceleration = 0.0;
    
        // Soft Limits
        public static final boolean kSoftLimitsEnabled = true;
        public static final double kPivotMinAngle = 0.0;
        public static final double kPivotMaxAngle = 0.0;
        // Position Conversion Factor for soft limits. Should be in units of Arm rotations, ie. the gear ratio. 
        // Periodically set the current position to the absolute position since it uses the internal encoder
        public static final double kPositionConversionFactor = 0.0;
    
        // Setpoints
        public static final double kStowPosition = 0.0;
        public static final double kFeedPosition = 0.0;
        public static final double kL1Position = 0.0;
        public static final double kL2L3Position = 0.0;
        public static final double kL4Position = 0.0;
    }
    
    public static class PincerConstants {
        // ID
        public static final int kPincerMotorId = 0;
        public static final int kIntakeMotorId = 0;
        public static final int kIntakePhotogateId = 0;
    
        // Motor Controller Settings
        public static final boolean kPincerMotorInverted = false;
        public static final IdleMode kPincerMotorIdleMode = IdleMode.kCoast;
        public static final int kPincerMotorSmartCurrentLimit = 40;

        public static final boolean kIntakeMotorInverted = false;
        public static final IdleMode kIntakeMotorIdleMode = IdleMode.kCoast;
        public static final int kIntakeMotorSmartCurrentLimit = 40;

        /** Current Threshold for determining if we have an algae */
        public static final int kIntakeAlgaeCurrentThreshold = 20;
    
        // PID Settings
        public static double kPincerP = 0.0;
        public static double kPincerI = 0.0;
        public static double kPincerD = 0.0;
        public static final FeedbackSensor kPincerMotorFeedbackSensor = FeedbackSensor.kAbsoluteEncoder;
        public static final double kPincerMotorMinOutput = -1;
        public static final double kPincerMotorMaxOutput = 1;
        public static final double kPincerMotorAbsoluteEncoderOffset = 0.0;
    
        // Soft Limits
        public static final boolean kSoftLimitsEnabled = true;
        public static final double kPincerMinAngle = 0.0;
        public static final double kPincerMaxAngle = 0.0;
        // Position Conversion Factor for soft limits. Should be in units of Arm rotations, ie. the gear ratio. 
        // Periodically set the current position to the absolute position since it uses the internal encoder
        public static final double kPositionConversionFactor = 0.0;

        // Pincer Setpoints
        public static final double kStowPosition = 0.0;
        public static final double kFunnelPosition = 0.0;
        public static final double kAlgaePosition = 0.0;

        // Intake Setpoints
        public static final double kIntakeSpeed = 0.0;
        public static final double kExhaustSpeed = 0.0;
    }

    public static class AscenderConstants {
        public static final int kAscenderMotorId = 0;
        public static final int kAscenderAbsEncoderId = 0;
    }

    public static class ElevatorConstants {
        public static final int kElevatorMotor1Id = 0;
        public static final int kElevatorMotor2Id = 0;
        public static final int kElevatorLaserCanId = 0;
        public static final int kElevatorLimitSwitchTopId = 0;
        public static final int kElevatorLimitSwitchBottomId = 0;

        public static final boolean kElevatorMotor1Inverted = false;
        public static final boolean kElevatorMotor2Inverted = false;
        public static final double kElevatorMotor1Offset = 0.0;
        public static final double kElevatorMotor2Offset = 0.0;

        public static final int kElevatorCurrentLimit = 40;

        public static final boolean kElevatorUseLaserCan = true;

        // Trapezoid Proflile parameters
        public static final double kElevatorMaxVelocity = 0.0;
        public static final double kElevatorMaxAcceleration = 0.0;

        // Spark built in encoder/controller PID constants
        public static final double kElevatorP = 0.0;
        public static final double kElevatorI = 0.0;
        public static final double kElevatorD = 0.0;
        public static final double kElevatorIZone = 0.0;
        public static final double kElevatorIMaxAccum = 0.0;

        // LaserCan PID constants
        public static final double kElevatorPLaserCan = 0.0;
        public static final double kElevatorILaserCan = 0.0;
        public static final double kElevatorDLaserCan = 0.0;

        // Feedforward constants
        public static final double kElevatorKs = 0.0;
        public static final double kElevatorKg = 0.0;
        public static final double kElevatorKv = 0.0;
        public static final double kElevatorKa = 0.0;
        public static final double kElevatorDtSeconds = 0.0;

        // Setpoints for targeting different levels, some other situations
        // For Spark built in encoder
        public static final double kElevatorSetpointL1 = 0.0;
        public static final double kElevatorSetpointL2 = 0.0;
        public static final double kElevatorSetpointL3 = 0.0;
        public static final double kElevatorSetpointL4 = 0.0;
        public static final double kElevatorSetpointFeed = 0.0;
        public static final double kElevatorSetpointStow = 0.0;

        // For LaserCan, meters
        public static final double kElevatorSetpointL1LaserCan = 0.0;
        public static final double kElevatorSetpointL2LaserCan = 0.0;
        public static final double kElevatorSetpointL3LaserCan = 0.0;
        public static final double kElevatorSetpointL4LaserCan = 0.0;
        public static final double kElevatorSetpointFeedLaserCan = 0.0; 
        public static final double kElevatorSetpointStowLaserCan = 0.0;
    }

    public static class GroundAlgaeConstants {
        public static final int kGroundAlgaeMotor1Id = 0;
        public static final int kGroundAlgaeMotor2Id = 0;
        public static final int kGroundAlgaeAbsEncoderId = 0;
        public static final int kGroundAlgaePhotogateId = 0;
    }

    public static class LEDConstants{
        public static final int kPwmPort = 0;
        public static final int kNumberOfLEDs = 40;

        // LED Patterns
        public static final LEDPattern kOrange = LEDPattern.solid(Color.kOrangeRed);
        public static final LEDPattern kGreen = LEDPattern.solid(Color.kGreen);
        public static final LEDPattern kGreenBlink = kGreen.blink(Milliseconds.of(100));
        public static final LEDPattern kJesuit = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGold, Color.kBlue);
        public static final LEDPattern kFastScrollingJesuit = kJesuit.scrollAtRelativeSpeed(Percent.per(Second).of(75));
        public static final LEDPattern kSlowScrollingJesuit = kJesuit.scrollAtRelativeSpeed(Percent.per(Second).of(25));
    }

}
