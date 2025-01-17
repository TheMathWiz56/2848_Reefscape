package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;

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
        public static final int kArmPivotMotorId = 0;
        public static final int kArmIntakeMotorId = 0;
        public static final int kArmPincerMotorId = 0;
        public static final int kArmPhotogateId = 0;
        public static final int kArmAbsEncoderId = 0;

        // Pivot Motor Controller Settings
        public static final boolean kArmPivotMotorInverted = false;
        public static final IdleMode kArmPivotMotorIdleMode = IdleMode.kCoast;
        public static final int kArmPivotMotorSmartCurrentLimit = 40;

        // Pincer Motor Controller Settings
        public static final boolean kArmPincerMotorInverted = false;
        public static final IdleMode kArmPincerMotorIdleMode = IdleMode.kCoast;
        public static final int kArmPincerMotorSmartCurrentLimit = 40;

        // Intake Motor Controller Settings
        public static final boolean kArmIntakeMotorInverted = false;
        public static final IdleMode kArmIntakeMotorIdleMode = IdleMode.kCoast;
        public static final int kArmIntakeMotorSmartCurrentLimit = 40;

        // Pivot PID/Feedforward Settings
        // removed final to tune, once tuned put back
        public static double kArmPivotP = 0.0;
        public static double kArmPivotI = 0.0;
        public static double kArmPivotD = 0.0;
        public static final FeedbackSensor kArmPivotMotorFeedbackSensor = FeedbackSensor.kAbsoluteEncoder;
        public static final double kArmPivotMotorMinOutput = -1;
        public static final double kArmPivotMotorMaxOutput = 1;
        public static final double kArmPivotMotorAbsoluteEncoderOffset = 0.0;
        public static double kArmPivotMotorks = 0.0;
        public static double kArmPivotMotorkg = 0.0;
        public static double kArmPivotMotorkv = 0.0;
        public static double kArmPivotMotorka = 0.0;
        public static double kArmPivotMotorMaxVelocity = 0.0;
        public static double kArmPivotMotorMaxAcceleration = 0.0;

        // Pincer PID Settings
        public static double kArmPincerP = 0.0;
        public static double kArmPincerI = 0.0;
        public static double kArmPincerD = 0.0;
        public static final FeedbackSensor kArmPincerMotorFeedbackSensor = FeedbackSensor.kAbsoluteEncoder;
        public static final double kArmPincerMotorMinOutput = -1;
        public static final double kArmPincerMotorMaxOutput = 1;
        public static final double kArmPincerMotorAbsoluteEncoderOffset = 0.0;

        // Intake PID Settings
        public static double kArmIntakeP = 0.0;
        public static double kArmIntakeI = 0.0;
        public static double kArmIntakeD = 0.0;
        public static final FeedbackSensor kArmIntakeMotorFeedbackSensor = FeedbackSensor.kAbsoluteEncoder;
        public static final double kArmIntakeMotorMinOutput = -1;
        public static final double kArmIntakeMotorMaxOutput = 1;
        public static final double kArmIntakeMotorAbsoluteEncoderOffset = 0.0;

        // Soft Limits
        public static final double kArmPivotMinAngle = 0.0;
        public static final double kArmPivotMaxAngle = 0.0;
        public static final double kArmPincerMinAngle = 0.0;
        public static final double kArmPincerMaxAngle = 0.0;

        // Setpoints
        public static final double kArmStowPosition = 0.0;
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
        public static final double kElevatorSetpointStow = 0.0;

        // For LaserCan
        public static final double kElevatorSetpointL1LaserCan = 0.0;
        public static final double kElevatorSetpointL2LaserCan = 0.0;
        public static final double kElevatorSetpointL3LaserCan = 0.0;
        public static final double kElevatorSetpointL4LaserCan = 0.0;
        public static final double kElevatorSetpointStowLaserCan = 0.0;
    }

    public static class GroundAlgaeConstants {
        public static final int kGroundAlgaeMotor1Id = 0;
        public static final int kGroundAlgaeMotor2Id = 0;
        public static final int kGroundAlgaeAbsEncoderId = 0;
        public static final int kGroundAlgaePhotogateId = 0;
    }

}
