package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.controller.HolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;


/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private RobotConfig config;

    

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;
  private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    AutoBuilder.configure(
      ()->this.getState().Pose, // Supplier of current robot pose
      this::seedFieldRelative,  // Consumer for seeding pose against auto
      ()->this.getState().speeds,
      (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    

    
  }
  public Command drivePath(Pose2d endPoint) {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
// The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
          endPoint
        );
        
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
          waypoints,
          constraints,
          null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
          new GoalEndState(0.0, endPoint.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
    try{
        // Load the path you want to follow using its name in the GUI
        

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }
/* 
  public Command followPathCommand(String pathName) {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathCommand(
                path,
                ()->this.getState().Pose, // Supplier of current robot pose
              ()->this.getState().speeds,
                (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new PPHolonomicDriveController(
                  new PIDConstants(5.0,5.0,5.0),
                  new PIDConstants(5.0,5.0,5.0)
                ),
                config, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
  */

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

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
    /*
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state
     */
    /*
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match
     */
    /*
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled
     */
    /*
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent((allianceColor) -> {
        this.setOperatorPerspectiveForward(
            allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                : BlueAlliancePerspectiveRotation);
        hasAppliedOperatorPerspective = true;
      });
    }
    updateOdometry();

    SmartDashboard.putString("Pose", getState().Pose.toString());
    // The pigeon reading is never changed from its boot zero
    SmartDashboard.putString("Pigeon Yaw", getPigeon2().getYaw().toString());
    SmartDashboard.putString("Adjusted Pigeon Yaw", new Pose2d(0,0,m_pigeon2.getRotation2d()).relativeTo(new Pose2d(0,0,m_fieldRelativeOffset)).getRotation().toString());
    
  }

  public void updateOdometry() {

      boolean useMegaTag2 = true; // set to false to use MegaTag1
      boolean doRejectUpdate = false;
      if (useMegaTag2 == false) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
          if (mt1.rawFiducials[0].ambiguity > .7) {
            doRejectUpdate = true;
          }
          if (mt1.rawFiducials[0].distToCamera > 3) {
            doRejectUpdate = true;
          }
        }
        if (mt1.tagCount == 0) {
          doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
              addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
        }
      } else if (useMegaTag2 == true) {
        LimelightHelpers.SetRobotOrientation("limelight", getState().Pose.getRotation().getDegrees(),
          0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (mt2 == null) { // in case mt2 returns a nullptr, need to figure out why this is happening
          doRejectUpdate = true;
        } else {
          if (Math.abs(m_pigeon2.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second,
                                                   // ignore vision updates
          {
            doRejectUpdate = true;
          }
          if (mt2.tagCount == 0) {
            doRejectUpdate = true;
          }
        }

        if (!doRejectUpdate) {
          addVisionMeasurement(mt2.pose, mt2.timestampSeconds); 
        }
      }


  }

}
