package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.kAngularVelocityConstant;
import static frc.robot.Constants.VisionConstants.kAngularVelocityMultiplier;
import static frc.robot.Constants.VisionConstants.kLinearVelocityConstant;
import static frc.robot.Constants.VisionConstants.kLinearVelocityMultiplier;

import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.reefData;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Util.reef;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // Field Widget in Elastic
    private static final Field2d m_field = new Field2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /** Swerve request to apply during field-centric PIDpath following */
    SwerveRequest.FieldCentric pathPIDRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    ProfiledPIDController pathPIDXController = new ProfiledPIDController(TunerConstants.pathPID_Translation_P, TunerConstants.pathPID_Translation_I, TunerConstants.pathPID_Translation_D, 
                                                                                    new TrapezoidProfile.Constraints(TunerConstants.pathPID_Translation_maxVx, TunerConstants.pathPID_Translation_MaxA));
    ProfiledPIDController pathPIDYController = new ProfiledPIDController(TunerConstants.pathPID_Translation_P, TunerConstants.pathPID_Translation_I, TunerConstants.pathPID_Translation_D, 
                                                                                    new TrapezoidProfile.Constraints(TunerConstants.pathPID_Translation_maxVy, TunerConstants.pathPID_Translation_MaxA));
    ProfiledPIDController pathPIDRotationController = new ProfiledPIDController(TunerConstants.pathPID_Rotation_P, TunerConstants.pathPID_Rotation_I, TunerConstants.pathPID_Rotation_D, 
                                                                                    new TrapezoidProfile.Constraints(TunerConstants.pathPID_Rotation_maxV, TunerConstants.pathPID_Rotation_MaxA));
    Timer timeToAlign = new Timer();
    private final Debouncer atGoalDebouncer = new Debouncer(TunerConstants.debounce_Time, DebounceType.kBoth);

    //Testing
    // Trapezoid profile for feedforward
    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        TunerConstants.pathPID_Translation_maxVx, TunerConstants.pathPID_Translation_MaxA));

    // Trapezoid profile states
    private TrapezoidProfile.State startState = new TrapezoidProfile.State();
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State currentState = new TrapezoidProfile.State();




    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private int flip_for_red = 1;
    private boolean isTrackingTagGoal = false;
    
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureDrivebase();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureDrivebase();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureDrivebase();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    

    //___________________________________________________ Custom Code ___________________________________________________


    private void configureDrivebase() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0), // 10
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0) // 7
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }

        // Configure PID controllers
        pathPIDXController.setTolerance(TunerConstants.pathPID_Translation_TolX);
        pathPIDYController.setTolerance(TunerConstants.pathPID_Translation_TolY);
        pathPIDRotationController.setTolerance(TunerConstants.pathPID_Rotation_Tol);
        pathPIDRotationController.enableContinuousInput(-Math.PI, Math.PI);

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            flip_for_red = -1;
        }
    }

    // Move to constants or another java file
    private Double[] Pose2dToDoubleArray(Pose2d pose){
        return new Double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }


    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });

        
        }
        
        // Vision measurements are added from the vision subsystem

        // Fused Pose Estimate Telemetry 
        Pose2d currentPose = getState().Pose;
        m_field.setRobotPose(new Pose2d(currentPose.getTranslation().getX(), currentPose.getTranslation().getY(), new Rotation2d(currentPose.getRotation().getRadians())));
        Double[] fusedPose = Pose2dToDoubleArray(currentPose);

        SmartDashboard.putNumber("Vision DriveBase Pose Estimate X", this.getState().Pose.getX());
        SmartDashboard.putNumber("Vision DriveBase Pose Estimate Y", this.getState().Pose.getY());
        SmartDashboard.putNumber("Vision DriveBase Linear Speed", Math.hypot(RobotContainer.getDrivetrain().getState().Speeds.vxMetersPerSecond, RobotContainer.getDrivetrain().getState().Speeds.vyMetersPerSecond));
        SmartDashboard.putNumber("Vision Drivebase Angular Speed", Math.abs(RobotContainer.getDrivetrain().getState().Speeds.omegaRadiansPerSecond));

        SmartDashboard.putBoolean("Test Path PID At Goal", pathPIDAtGoal());
        SmartDashboard.putNumber("Test Path PID Error", pathPIDRotationController.getPositionError());
    }


    // ___________________________________________________ Vision Code ___________________________________________________
    
    /**
     * Resets the robot's Odometry pose estimate to the best current mt1 pose estimate.
     * <p>Can also use a known reference like a wall to zero the Pigeon (most important thing for mt2 is having an accurate yaw reading)
     * @param forceUpdate Override the tag area requirement
     */
    public void resetToVision(){
        LimelightHelpers.PoseEstimate poseEstimate = RobotContainer.getVision().getVisionPoseEstimateMT1(); // Might be able to switch to mt1 or 2. Needs testing if want to change
        SmartDashboard.putBoolean("Is Getting null pose estimate", poseEstimate != null);
        if (poseEstimate != null) {
            resetPose(poseEstimate.pose);         
        }
    }

    public Command stop(){
        return runOnce(() -> this.setControl(TunerConstants.stopRequest));
    }

    /**
     * Creates a command that moves the robot to the position of the AprilTag 
     * detected by the Limelight camera, adjusted by the left branch transformation.
     * 
     * @return A {@link Command} that moves the robot to the transformed position of the detected tag. 
     *         If no tag is visible, a command is returned that logs the absence of a tag.
     */
    private Command pathPIDToTagLeft(int ID){
        SmartDashboard.putNumber("Tag ID used", ID);
        SmartDashboard.putString("Path PID to", reef.tagPoseAndymarkMap.get(ID).transformBy(TunerConstants.rightBranch).toString());

        if (ID != -1) {
            return this.pathPIDTo(reef.tagPoseAndymarkMap.get(ID).transformBy(TunerConstants.leftBranch),
                    reef.tagPoseAndymarkMap.get(ID), false);
        }
        return this.runOnce(() -> SmartDashboard.putBoolean("No Tag at pathPID", true));
    }

    private Command pathPIDToTagMiddle(int ID) {
        SmartDashboard.putNumber("Tag ID used", ID);

        if (ID != -1) {
            return this.pathPIDTo(reef.tagPoseAndymarkMap.get(ID).transformBy(TunerConstants.reefAlgae),
                    reef.tagPoseAndymarkMap.get(ID), true);
        }
        return this.runOnce(() -> SmartDashboard.putBoolean("No Tag at pathPID", true));

    }

    /**
     * Creates a command that moves the robot to the position of the AprilTag
     * detected by the Limelight camera, adjusted by the right branch
     * transformation.
     * 
     * @return A {@link Command} that moves the robot to the transformed position of
     *         the detected tag.
     *         If no tag is visible, a command is returned that logs the absence of
     *         a tag.
     */
    private Command pathPIDToTagRight(int ID) {
        SmartDashboard.putNumber("Tag ID used", ID);
        SmartDashboard.putString("Path PID to",
                reef.tagPoseAndymarkMap.get(ID).transformBy(TunerConstants.rightBranch).toString());

        if (ID != -1) {
            return this.pathPIDTo(reef.tagPoseAndymarkMap.get(ID).transformBy(TunerConstants.rightBranch),
                    reef.tagPoseAndymarkMap.get(ID), false);
        }
        return this.runOnce(() -> SmartDashboard.putBoolean("No Tag at pathPID", true));
    }

    public Command pathPIDToTagMiddleSelect(){
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(17, this.pathPIDToTagMiddle(17)),
                Map.entry(18, this.pathPIDToTagMiddle(18)),
                Map.entry(19, this.pathPIDToTagMiddle(19)),
                Map.entry(20, this.pathPIDToTagMiddle(20)),
                Map.entry(21, this.pathPIDToTagMiddle(21)),
                Map.entry(22, this.pathPIDToTagMiddle(22)),
                Map.entry(6, this.pathPIDToTagMiddle(6)),
                Map.entry(7, this.pathPIDToTagMiddle(7)),
                Map.entry(8, this.pathPIDToTagMiddle(8)),
                Map.entry(9, this.pathPIDToTagMiddle(9)),
                Map.entry(10, this.pathPIDToTagMiddle(10)),
                Map.entry(11, this.pathPIDToTagMiddle(11)))
        , () -> RobotContainer.getVision().getTag());
    }

    public Command pathPIDToTagRightSelect(){
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(17, this.pathPIDToTagRight(17)),
                Map.entry(18, this.pathPIDToTagRight(18)),
                Map.entry(19, this.pathPIDToTagRight(19)),
                Map.entry(20, this.pathPIDToTagRight(20)),
                Map.entry(21, this.pathPIDToTagRight(21)),
                Map.entry(22, this.pathPIDToTagRight(22)), 
                Map.entry(6, this.pathPIDToTagRight(6)),
                Map.entry(7, this.pathPIDToTagRight(7)),
                Map.entry(8, this.pathPIDToTagRight(8)),
                Map.entry(9, this.pathPIDToTagRight(9)),
                Map.entry(10, this.pathPIDToTagRight(10)),
                Map.entry(11, this.pathPIDToTagRight(11)))
        , () -> RobotContainer.getVision().getTag());
    }

    public Command pathPIDToTagLeftSelect(){
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(17, this.pathPIDToTagLeft(17)),
                Map.entry(18, this.pathPIDToTagLeft(18)),
                Map.entry(19, this.pathPIDToTagLeft(19)),
                Map.entry(20, this.pathPIDToTagLeft(20)),
                Map.entry(21, this.pathPIDToTagLeft(21)),
                Map.entry(22, this.pathPIDToTagLeft(22)),
                Map.entry(6, this.pathPIDToTagLeft(6)),
                Map.entry(7, this.pathPIDToTagLeft(7)),
                Map.entry(8, this.pathPIDToTagLeft(8)),
                Map.entry(9, this.pathPIDToTagLeft(9)),
                Map.entry(10, this.pathPIDToTagLeft(10)),
                Map.entry(11, this.pathPIDToTagLeft(11)))
        , () -> RobotContainer.getVision().getTag());
    }


    private boolean pose2dSameYSign(Pose2d pose1, Pose2d pose2){
        return (pose1.getY() > 0 && pose2.getY() > 0 )|| (pose1.getY() < 0 && pose2.getY() < 0);
    }
    

    /**
     * Creates a command that moves the robot to the specified {@link Pose2d} using PID controllers 
     * for X, Y, and rotation. The command runs until all PID controllers reach their goals, 
     * as determined by the debouncer.
     * 
     * @param goalPose The target {@link Pose2d} the robot should move to.
     * @return A {@link Command} that moves the robot to the specified pose.
     */
    private Command pathPIDTo(Pose2d goalPose, Pose2d tagPose, boolean isAlgae){
        return this.startRun(()->{
            timeToAlign.reset();
            timeToAlign.start();

            Pose2d currentFieldPose2d = this.getState().Pose;
            Pose2d currentTagPose2d = currentFieldPose2d.relativeTo(tagPose);
            Pose2d goalTagPose2d = goalPose.relativeTo(tagPose);

            
            if (Math.abs(currentTagPose2d.getTranslation().getY()) > TunerConstants.tagYShiftLimit && !isAlgae && pose2dSameYSign(goalTagPose2d, currentTagPose2d)){
                isTrackingTagGoal = false;
                double Y0 = currentTagPose2d.getTranslation().getY();
                double shift = TunerConstants.maxTagYShift * ( -1 * Y0 / Math.abs(Y0));
                goalTagPose2d = new Pose2d(goalTagPose2d.getTranslation().plus(new Translation2d(0.0,shift)), goalTagPose2d.getRotation());
                pathPIDYController.setTolerance(TunerConstants.pathPID_Translation_TolYShift);
                pathPIDXController.atGoal(); //Think this is not necessary
            }
            else{
                isTrackingTagGoal = true;
            }

            pathPIDXController.reset(currentTagPose2d.getX()); //can reset by giving the controller the current position and velocity
            pathPIDYController.reset(currentTagPose2d.getY());
            pathPIDRotationController.reset(currentTagPose2d.getRotation().getRadians());

            pathPIDXController.setGoal(goalTagPose2d.getX());
            pathPIDYController.setGoal(goalTagPose2d.getY());
            pathPIDRotationController.setGoal(goalTagPose2d.getRotation().getRadians());
        
            }, () -> {

                
                if (pathPIDYController.atGoal() && !isTrackingTagGoal){
                    pathPIDYController.setGoal(goalPose.relativeTo(tagPose).getY());

                    pathPIDYController.setTolerance(TunerConstants.pathPID_Translation_TolY);
                    isTrackingTagGoal = true;
                }

                Pose2d currentFieldPose2d = this.getState().Pose;
                Pose2d currentTagPose2d = currentFieldPose2d.relativeTo(tagPose);

                Translation2d positionPID = new Translation2d(pathPIDXController.calculate(currentTagPose2d.getX()), pathPIDYController.calculate(currentTagPose2d.getY())).rotateBy(tagPose.getRotation());
                Translation2d fieldVelocity = new Translation2d(pathPIDXController.getSetpoint().velocity, pathPIDYController.getSetpoint().velocity).rotateBy(tagPose.getRotation());
                
                fieldVelocity = fieldVelocity.plus(positionPID);

                pathPIDRequest
                    .withVelocityX(fieldVelocity.getX() * flip_for_red)
                    .withVelocityY(fieldVelocity.getY() * flip_for_red)
                    .withRotationalRate(pathPIDRotationController.calculate(currentTagPose2d.getRotation().getRadians()))
                    .withDeadband(TunerConstants.pathPID_Translation_Deadband)
                    .withRotationalDeadband(TunerConstants.pathPID_Rotation_Deadband);

                this.setControl(pathPIDRequest);
                
                /*
                SmartDashboard.putNumber("X PID Position Error", pathPIDXController.getPositionError());
                SmartDashboard.putNumber("X PID Velocity Error", pathPIDXController.getVelocityError());
                SmartDashboard.putNumber("X PID Velocity setpoint", pathPIDXController.getSetpoint().velocity);
                SmartDashboard.putNumber("X PID Field Velocity setpoint", fieldVelocity.getX() * flip_for_red);
                SmartDashboard.putNumber("X PID Position Setpoint", pathPIDXController.getSetpoint().position);
                SmartDashboard.putNumber("X PID Position PV", currentTagPose2d.getX());
                SmartDashboard.putNumber("X PID Output", positionPID.getX());*/

            }).until(() -> pathPIDAtGoal()).withName("PathPIDTo").andThen(() -> {
                timeToAlign.stop();
                SmartDashboard.putNumber("Time To Align", timeToAlign.get()); }, this);
    }

    public Command testPathPIDTo (Pose2d goalPose, Pose2d tagPose){
        return this.startRun(()->{
            Pose2d currentFieldPose2d = this.getState().Pose;
            Pose2d currentTagPose2d = currentFieldPose2d.relativeTo(tagPose);
            Pose2d goalTagPose2d = goalPose.relativeTo(tagPose);

            pathPIDXController.reset(currentTagPose2d.getX()); //can reset by giving the controller the current position and velocity
            pathPIDYController.reset(currentTagPose2d.getY());
            pathPIDRotationController.reset(currentTagPose2d.getRotation().getRadians(), 0);

            pathPIDYController.setTolerance(TunerConstants.pathPID_Translation_TolY);
            isTrackingTagGoal = true;

            pathPIDXController.setGoal(goalTagPose2d.getX());
            pathPIDYController.setGoal(goalTagPose2d.getY());
            pathPIDRotationController.setGoal(goalTagPose2d.getRotation().getRadians());

            SmartDashboard.putNumber("Test Path PID Goal", pathPIDRotationController.getGoal().position);
        
            }, () -> {
                Pose2d currentFieldPose2d = this.getState().Pose;
                Pose2d currentTagPose2d = currentFieldPose2d.relativeTo(tagPose);

                Translation2d positionPID = new Translation2d(pathPIDXController.calculate(currentTagPose2d.getX()), pathPIDYController.calculate(currentTagPose2d.getY())).rotateBy(tagPose.getRotation());
                Translation2d fieldVelocity = new Translation2d(pathPIDXController.getSetpoint().velocity, pathPIDYController.getSetpoint().velocity).rotateBy(tagPose.getRotation());
                
                fieldVelocity = fieldVelocity.plus(positionPID);

                pathPIDRequest
                    .withVelocityX(fieldVelocity.getX() * flip_for_red)
                    .withVelocityY(fieldVelocity.getY() * flip_for_red)
                    .withRotationalRate(pathPIDRotationController.calculate(currentTagPose2d.getRotation().getRadians()))
                    .withDeadband(TunerConstants.pathPID_Translation_Deadband)
                    .withRotationalDeadband(TunerConstants.pathPID_Rotation_Deadband);

                this.setControl(pathPIDRequest);

            }).until(() -> pathPIDAtGoal()).withName("PathPIDTo");
    }

    /**
     * Checks if the PID path follower has been at the goal for the specified debouncer time.
     * 
     * @return {@code true} if the PID controllers for X, Y, and rotation have all been at the goal 
     *         for at least {@code 0.5} seconds (the debouncer time), otherwise {@code false}.
     */
    public boolean pathPIDAtGoal (){
        return atGoalDebouncer.calculate(pathPIDXController.atGoal() && pathPIDYController.atGoal() && pathPIDRotationController.atGoal() && isTrackingTagGoal);
    }
}