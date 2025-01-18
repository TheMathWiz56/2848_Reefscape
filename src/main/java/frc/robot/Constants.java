package frc.robot;

import edu.wpi.first.wpilibj.DutyCycle;

public final class Constants {

    // Constants relating to the drivetrain are probably in TunerConstants

    public static class ControllerConstants {
        public static final int kDriverController = 0;
        // One of the operator controllers is going to be for emergencies if all else
        // fails
        public static final int kOperatorController1 = 1;
        public static final int kOperatorController2 = 2;
    }

    public static class ArmConstants {
        public static final int kArmPivotMotorId = 0;
        public static final int kArmWheelsMotorId = 0;
        public static final int kArmPhotogateId = 0;
        public static final int kArmAbsEncoderId = 0;

        public static final double kArmP = 0.0;
        public static final double kArmI = 0.0;
        public static final double kArmD = 0.0;
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

        // For LaserCan, meters
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
