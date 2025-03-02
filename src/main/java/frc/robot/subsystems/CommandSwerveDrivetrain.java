package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

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

    // April tag variables
    private static boolean useMegaTag2 = true; // set to false to use MegaTag1. Should test to see which one works better, 1 or 2? Or if they can be combined/we switch between them based on some conditions
    private static boolean doRejectUpdate = false;
    private static String limelightUsed;
    private static LimelightHelpers.PoseEstimate LLPoseEstimate;
    //Get average tag areas (percentage of image), Choose the limelight with the highest average tag area
    private static double limelightFrontAvgTagArea = 0;
    private static double limelightBackAvgTagArea = 0;

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
                                                                                    new TrapezoidProfile.Constraints(TunerConstants.pathPID_Translation_maxV, TunerConstants.pathPID_Translation_MaxA));
    ProfiledPIDController pathPIDYController = new ProfiledPIDController(TunerConstants.pathPID_Translation_P, TunerConstants.pathPID_Translation_I, TunerConstants.pathPID_Translation_D, 
                                                                                    new TrapezoidProfile.Constraints(TunerConstants.pathPID_Translation_maxV, TunerConstants.pathPID_Translation_MaxA));
    ProfiledPIDController pathPIDRotationController = new ProfiledPIDController(TunerConstants.pathPID_Rotation_P, TunerConstants.pathPID_Rotation_I, TunerConstants.pathPID_Rotation_D, 
                                                                                    new TrapezoidProfile.Constraints(TunerConstants.pathPID_Rotation_maxV, TunerConstants.pathPID_Rotation_MaxA));

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();


    
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
        configureAutoBuilder();
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
        configureAutoBuilder();
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
        configureAutoBuilder();
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


    private void configureAutoBuilder() {
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
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
        
        NamedCommands.registerCommand("Score_L2", new WaitCommand(2)); // Placeholder for now
        NamedCommands.registerCommand("Score_L4", new WaitCommand(2)); // Placeholder for now

        // Configure PID controllers
        pathPIDXController.setTolerance(TunerConstants.pathPID_Translation_Tol);
        pathPIDYController.setTolerance(TunerConstants.pathPID_Translation_Tol);
        pathPIDRotationController.setTolerance(TunerConstants.pathPID_Rotation_Tol);
        pathPIDRotationController.enableContinuousInput(-Math.PI, Math.PI);
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
        
        updateOdometry();

        // Fused Pose Estimate Telemetry 
        Pose2d currentPose = getState().Pose;
        m_field.setRobotPose(new Pose2d(currentPose.getTranslation().getX(), currentPose.getTranslation().getY(), new Rotation2d(currentPose.getRotation().getRadians())));
        Double[] fusedPose = Pose2dToDoubleArray(currentPose);
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putNumberArray("Fused PoseDBL", fusedPose);

        Command currentCommand = this.getCurrentCommand();

        if (currentCommand != null){
            SmartDashboard.putString("Drivebase Current Command", currentCommand.getName());
        }
        
    }


    // ___________________________________________________ Vision Code ___________________________________________________
    
    /**
     * Resets the robot's Odometry pose estimate to the best current mt1 pose estimate.
     * <p>Can also use a known reference like a wall to zero the Pigeon (most important thing for mt2 is having an accurate yaw reading)
     * @param forceUpdate Override the tag area requirement
     */
    public void resetToVision(boolean forceUpdate){
        chooseLL();
        LimelightHelpers.PoseEstimate poseEstimate = getLLMegaTEstimate(false); // Might be able to switch to mt1 or 2. Needs testing if want to change

        if (poseEstimate != null) {
            if (forceUpdate || limelightBackAvgTagArea > 3){
                resetPose(poseEstimate.pose);
            }          
        }
    }

    /**
     * Polls the limelights for a pose estimate and uses the pose estimator Kalman filter to fuse the best Limelight pose estimate
     * with the odometry pose estimate
     */
    private void updateOdometry() {
        chooseLL();
        LLPoseEstimate = getLLMegaTEstimate(true); 

        if (LLPoseEstimate != null) {
            // needs to be converted to a current time timestamp for it to be combined properly with the odometry pose estimate
            SmartDashboard.putNumber("Odometry Update Timestamp", Utils.fpgaToCurrentTime(LLPoseEstimate.timestampSeconds)); 
            SmartDashboard.putNumberArray("Incoming Pose Estimate", Pose2dToDoubleArray(LLPoseEstimate.pose));
            addVisionMeasurement(LLPoseEstimate.pose, Utils.fpgaToCurrentTime(LLPoseEstimate.timestampSeconds), TunerConstants.visionStandardDeviation);
        }
    }

    /**
     * Uses the autobuilder and PathPlanner's navigation grid to pathfind to a pose in real time
     * 
     * @param pose Pose to pathfind to
     * @param endVelocity Velocity at target pose
     */
    public Command pathPlanTo(Pose2d pose, LinearVelocity endVelocity){
        return AutoBuilder.pathfindToPose(pose, TunerConstants.oTF_Constraints, endVelocity);
    }

    /**
     * @param useMegaTag2 Boolean to use mt2 or mt1
     * @return Valid pose estimate or null
     */
    private LimelightHelpers.PoseEstimate getLLMegaTEstimate(boolean useMegaTag2){
        doRejectUpdate = false;
        LimelightHelpers.PoseEstimate poseEstimate = new LimelightHelpers.PoseEstimate();

        LimelightHelpers.SetRobotOrientation("limelight-front", getState().Pose.getRotation().getDegrees(),
        0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-back", getState().Pose.getRotation().getDegrees(),
        0, 0, 0, 0, 0);
        


        if (useMegaTag2 == false) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightUsed);

            if (poseEstimate == null){
                doRejectUpdate = true;
            }
            else{
                if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) {
                    if (poseEstimate.rawFiducials[0].ambiguity > .7) {
                        doRejectUpdate = true;
                    }
                    if (poseEstimate.rawFiducials[0].distToCamera > 3) {
                        doRejectUpdate = true;
                    }
                    }
                    if (poseEstimate.tagCount == 0) {
                    doRejectUpdate = true;
                    }
            }
        }
        else{
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightUsed);
            if (poseEstimate == null) {
                doRejectUpdate = true;
            } 
            else {
                if (Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720){ // if our angular velocity is greater than 720 degrees per second,
                    doRejectUpdate = true;
                }
                if (poseEstimate.tagCount == 0) {
                    doRejectUpdate = true;
                }
            }
        }

        if (doRejectUpdate){
            return null;
        }
        else{
            return poseEstimate;
        }
    }

    /**
     * Updates the currently used limelight based on which limelight has the largest average tag area.
     */
    private static void chooseLL(){
        limelightFrontAvgTagArea = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose").getDoubleArray(new double[11])[10];
        limelightBackAvgTagArea = NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("botpose").getDoubleArray(new double[11])[10];
        SmartDashboard.putNumber("Front Limelight Tag Area", limelightFrontAvgTagArea);
        SmartDashboard.putNumber("Back Limelight Tag Area", limelightBackAvgTagArea);   

        double translationSTD = TunerConstants.std02; // safe value
        if(limelightFrontAvgTagArea > limelightBackAvgTagArea){
            limelightUsed = "limelight-front";
            translationSTD = TunerConstants.getVisionStd(limelightFrontAvgTagArea);
        }
        else{
            limelightUsed = "limelight-back";
            translationSTD = TunerConstants.getVisionStd(limelightBackAvgTagArea);
                
        }
        
        TunerConstants.visionStandardDeviation = VecBuilder.fill(translationSTD, translationSTD, 9999999); // Don't trust yaw, rely on Pigeon

        SmartDashboard.putNumberArray("Vision Standard Deviations", TunerConstants.visionStandardDeviation.getData());
        SmartDashboard.putString("Limelight Used", limelightUsed);
    }

    public boolean LLHasTag(){
        return LimelightHelpers.getTargetCount(limelightUsed) > 0;
    }


    // Increase rotation deadzone and have setpoint inside the reef so the robot pushes against the reef to align
    // move camera so it's always in sight of april tag
    // apply rotation deadband
    public Command pathPIDTo(Pose2d pose){
        return this.startRun(()->{
            Pose2d currentPose2d = this.getState().Pose;

            pathPIDXController.reset(currentPose2d.getX());
            pathPIDYController.reset(currentPose2d.getY());
            pathPIDRotationController.reset(currentPose2d.getRotation().getRadians());

            pathPIDXController.setGoal(pose.getX());
            pathPIDYController.setGoal(pose.getY());
            pathPIDRotationController.setGoal(pose.getRotation().getRadians());
        
            }, () -> {
                Pose2d currentPose2d = this.getState().Pose;

                /*
                pathPIDRequest
                    .withVelocityX(pathPIDXController.calculate(currentPose2d.getX()) + pathPIDXController.getSetpoint().velocity)
                    .withVelocityY(pathPIDYController.calculate(currentPose2d.getY())+ pathPIDYController.getSetpoint().velocity)
                    .withRotationalRate(pathPIDRotationController.calculate(currentPose2d.getRotation().getRadians()) + pathPIDRotationController.getSetpoint().velocity)
                    .withDeadband(0.05)
                    .withRotationalDeadband(0.075); // + pathPIDRotationController.getSetpoint().velocity */

                pathPIDRequest
                    .withVelocityX(pathPIDXController.calculate(currentPose2d.getX()))
                    .withVelocityY(pathPIDYController.calculate(currentPose2d.getY()))
                    .withRotationalRate(pathPIDRotationController.calculate(currentPose2d.getRotation().getRadians()))
                    .withDeadband(0.05)
                    .withRotationalDeadband(0.02); // + pathPIDRotationController.getSetpoint().velocity

                this.setControl(pathPIDRequest);

                SmartDashboard.putNumber("X PID Position Error", pathPIDXController.getPositionError());
                SmartDashboard.putNumber("X PID Velocity Error", pathPIDXController.getVelocityError());
                SmartDashboard.putNumber("X PID Velocity setpoint", pathPIDXController.getSetpoint().velocity);
                SmartDashboard.putNumber("X PID Output", pathPIDXController.calculate(currentPose2d.getX()));

                SmartDashboard.putNumber("Y PID Position Error", pathPIDYController.getPositionError());
                SmartDashboard.putNumber("Y PID Velocity Error", pathPIDYController.getVelocityError());
                SmartDashboard.putNumber("Y PID Velocity setpoint", pathPIDYController.getSetpoint().velocity);
                SmartDashboard.putNumber("Y PID Output", pathPIDYController.calculate(currentPose2d.getY()));

                SmartDashboard.putNumber("Roation PID Position", currentPose2d.getRotation().getRadians());
                SmartDashboard.putNumber("Rotation PID Position Error", pathPIDRotationController.getPositionError());
                SmartDashboard.putNumber("Rotation PID Velocity Error", pathPIDRotationController.getVelocityError());
                SmartDashboard.putNumber("Roation PID Velocity Setpoint", pathPIDRotationController.getSetpoint().velocity);
                SmartDashboard.putNumber("Roation PID Position Setpoint", pathPIDRotationController.getSetpoint().position);
                SmartDashboard.putNumber("Rotation PID Output", pathPIDRotationController.calculate(currentPose2d.getRotation().getRadians()));

                SmartDashboard.putBoolean("X PID At Goal", pathPIDXController.atGoal());
                SmartDashboard.putBoolean("Y PID At Goal", pathPIDYController.atGoal());
                SmartDashboard.putBoolean("Rotation PID At Goal", pathPIDRotationController.atGoal());

                SmartDashboard.putNumber("X Total Output", pathPIDXController.calculate(currentPose2d.getX()) + pathPIDXController.getSetpoint().velocity);
                SmartDashboard.putNumber("Y Total Output", pathPIDYController.calculate(currentPose2d.getY())+ pathPIDYController.getSetpoint().velocity);
                SmartDashboard.putNumber("Rotation Total Output", pathPIDRotationController.calculate(currentPose2d.getRotation().getRadians()) + pathPIDRotationController.getSetpoint().velocity);

            }).withName("PathPIDTo");

            // .until(() -> pathPIDXController.atGoal() && pathPIDYController.atGoal() && pathPIDRotationController.atGoal())
    }

    public boolean pathPADAtGoal (){
        return pathPIDXController.atGoal() && pathPIDYController.atGoal() && pathPIDRotationController.atGoal();
    }
}