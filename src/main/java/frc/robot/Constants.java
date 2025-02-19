package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Util.fieldPoint;
import frc.robot.Util.fieldPoly;

public final class Constants {

    /**
     * I don't think we need this - the description of the function configureAsync()
     * says it will configure without waiting for a response, so the regular
     * config() should wait automatically
     * Delays the program to allow the motor controller to burn the configure to
     * memory
     */
    
    
        

    //util values for the reef
    public static class reef{
        /*possible reef values l for left, r for right, L for level */
        public static enum reefSide{
            LEFT,RIGHT
        }
        public static enum reefLs{
            lL4,
            lL3,
            lL2,
            lL1,
            rL4,
            rL3,
            rL2,
            rL1,
            STOW,
            NONE
        }

        public static final Map<Integer, Integer> rMap = new HashMap<>(){{
            put(9,1);
            put(10,2);
            put(11,3);
            put(12,4);
            put(13,5);
            put(14,6);
        }};

        public static final Map<Integer,reefLs> lMap = new HashMap<>() {{
            put(1, reefLs.lL4);
            put(2, reefLs.lL3);
            put(3, reefLs.lL2);
            put(4, reefLs.lL1);
            put(5, reefLs.rL4);
            put(6, reefLs.rL3);
            put(7, reefLs.rL2);
            put(8, reefLs.rL1);
            put(22, reefLs.STOW);
            put(19,reefLs.STOW);
        }};
        
        //zones for auto driving around the reef
        public static final class driveZones{
            
            public static final fieldPoly reef1Zone = new fieldPoly(
                new fieldPoint(5,0),
                new fieldPoint(-5,0),
                new fieldPoint(-5,-5),
                new fieldPoint(5,-5)
            );


            public static final Map<Integer,fieldPoly> fieldPolyList = new HashMap<>(){{
                put(1,reef1Zone);
            }};
        }

}

    // Constants relating to the drivetrain are probably in TunerConstants

    public static void kMotorBurnDelay() {
        Timer.delay(0.1);
    }

    public static double deadzone(double input, double zone) {
        return Math.abs(input) >= zone ? input : 0.0;
    }

    /**
     * Adjusts the input reference to ensure it stays within the specified range.
     *
     * @param input the desired reference value
     * @param min   the minimum allowable value
     * @param max   the maximum allowable value
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
        public static final int kPivotMotorId = 21;

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
        public static final boolean kSoftLimitsEnabled = false;
        public static final double kPivotMinAngle = 1.0;
        public static final double kPivotMaxAngle = 2.0;
        // Position Conversion Factor for soft limits. Should be in units of Arm
        // rotations, ie. the gear ratio.
        // Periodically set the current position to the absolute position since it uses
        // the internal encoder
        public static final double kPositionConversionFactor = 1;

        // Setpoints
        // For now: L1 is facing almost straight up, L2L3 is diagonal, L4 is facing straight-on
        public static final double kStowPosition = 0.0;
        public static final double kFeedPosition = 0.0;
        public static final double kL1Position = 0.1;
        public static final double kL2L3Position = 0.2;
        public static final double kL4Position = 0.325;
    }

    public static class PincerConstants {
        // ID
        public static final int kPincerMotorId = 22; // Claw
        public static final int kIntakeMotorId = 23; // Wheels
        //public static final int kIntakePhotogateId = 0;

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
        public static final boolean kSoftLimitsEnabled = false;
        public static final double kPincerMinAngle = 1.0;
        public static final double kPincerMaxAngle = 2.0;
        // Position Conversion Factor for soft limits. Should be in units of Arm
        // rotations, ie. the gear ratio.
        // Periodically set the current position to the absolute position since it uses
        // the internal encoder
        public static final double kPositionConversionFactor = 1.0;

        // Pincer Setpoints
        public static final double kStowPosition = 0.0;
        public static final double kFunnelPosition = 0.0;
        public static final double kAlgaePosition = 0.0;

        // Intake Setpoints
        public static final double kIntakeSpeed = -0.1;
        public static final double kExhaustSpeed = 0.1;
    }

    public static class AscenderConstants {
        public static final int kAscenderMotorId = 25;
        public static final int kAscenderLimitSwitchId = 0;
    }

    public static class ElevatorConstants {
        public static final int kMotorId = 20;
        public static final int kLimitSwitchBottomId = 1;

        public static final InvertedValue kMotorInverted = InvertedValue.CounterClockwise_Positive;
        public static final double kMotorEncoderOffset = 0.0;
        public static final NeutralModeValue kMotorIdleMode = NeutralModeValue.Coast;

        public static final int kCurrentLimit = 50;

        // Trapezoid Proflile parameters
        public static final double kMaxVelocity = 30.0;
        public static final double kMaxAcceleration = 15.0;

        // Spark built in encoder/controller PID constants
        public static final double kP = 2.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIZone = 0.0;
        public static final double kIMaxAccum = 0.0;

        // Feedforward constants
        public static final double kFeedforwardKs = 0.0;
        public static final double kFeedforwardKg = -0.5;
        public static final double kFeedforwardKv = 0.25;
        public static final double kFeedforwardKa = 0.0;
        public static final double kFeedforwardDtSeconds = 0.0;

        // Setpoints for targeting different levels, some other situations
        // For Spark built in encoder
        public static final double kSetpointL1 = 0.0;
        public static final double kSetpointL2 = -23.044921875;        
        public static final double kSetpointL3 = -32.32275390625;
        public static final double kSetpointL4 = -32.0;
        public static final double kSetpointFeed = 0.0;
        public static final double kSetpointStow = 0.0;

        public static final double kSetpointAlgaeLow = -23.0;
        public static final double kSetpointAlgaeHigh = -30.0;
        public static final double kSetpointnet = -32.0;

        public static final boolean kUseCurrentForZeroing = false;
        public static final double kZeroingCurrent = 20.0;
    }

    public static class GroundAlgaePivotConstants {
        public static final int kMotorId = 0;

        public static final boolean kMotorInverted = false;
        public static final IdleMode kMotorIdleMode = IdleMode.kCoast;
        public static final double kMotorEncoderOffset = 0.0;

        public static final int kCurrentLimit = 40;

        public static final double kMotorPositionConversionFactor = 1;
        public static final double kMotorVelocityConversionFactor = 1;

        // Trapezoid Proflile parameters
        public static final double kMaxVelocity = 0.0;
        public static final double kMaxAcceleration = 0.0;

        // PID for spark controller
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward constants
        public static final double kFeedforwardKs = 0.0;
        public static final double kFeedforwardKg = 0.0;
        public static final double kFeedforwardKv = 0.0;
        public static final double kFeedforwardKa = 0.0;
        public static final double kFeedforwardDtSeconds = 0.0;

        // Setpoints
        public static final double kSetpointStow = 0.0;
        public static final double kSetpointIntake = 0.0;
        public static final double kSetpointScore = 0.0;
        
        public static final boolean kForwardSoftLimitEnabled = false;
        public static final double kForwardSoftLimit = 0.0;
        public static final boolean kReverseSoftLimitEnabled = true;
        public static final double kReverseSoftLimit = -40.5;
    }

    public static class GroundAlgaeWheelsConstants {
        public static final int kMotorId = 0;
        public static final int kPhotogateId = 0;

        public static final boolean kMotorInverted = false;
        public static final IdleMode kMotorIdleMode = IdleMode.kCoast;
        public static final double kMotorEncoderOffset = 0.0;

        public static final int kCurrentLimit = 40;

        public static final double kIntakeSpeed = 0.0;
        public static final double kExhaustSpeed = 0.0;

    }

    public static class LEDConstants {
        public static final int kPwmPort = 0;
        public static final int kNumberOfLEDs = 150;

        // LED Patterns
        public static final LEDPattern kOrange = LEDPattern.solid(Color.kOrangeRed);
        public static final LEDPattern kGreen = LEDPattern.solid(Color.kGreen);
        public static final LEDPattern kGreenBlink = kGreen.blink(Milliseconds.of(100));
        public static final LEDPattern kJesuit = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGold,
                Color.kBlue);
        public static final LEDPattern kFastScrollingJesuit = kJesuit.scrollAtRelativeSpeed(Percent.per(Second).of(75));
        public static final LEDPattern kSlowScrollingJesuit = kJesuit.scrollAtRelativeSpeed(Percent.per(Second).of(25));
        public static final LEDPattern kRainbow = LEDPattern.rainbow(255,128);

        // Scrolling rainbow from WPILib docs reference
        /*
        // Our LED strip has a density of 60 LEDs per meter
        public  static final Distance kLedSpacing = Meters.of(1 / 60.0);

        
        // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
        // of 1 meter per second.
        public static final LEDPattern kScrollingRainbow =
            kRainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
        
            */
            }             

}
