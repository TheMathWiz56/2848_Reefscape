package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

import lombok.Getter;
import lombok.Setter;

/**
 * Vision subsystem for handling LimeLight interfacing
 */
public class Vision extends SubsystemBase {

    @Getter @Setter private boolean useMegaTag2 = false;
    private String[] cameraList;
    @Getter @Setter private String bestLimeLight = "";
    @Getter @Setter private double translationStdDev = kInvalidStandardDeviation;
    @Getter @Setter private double rotationStdDev = kInvalidStandardDeviation;
    @Getter private Matrix<N3, N1> visionStandardDeviation = VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    @Getter private LimelightHelpers.PoseEstimate visionPoseEstimate = null;
    @Getter private LimelightHelpers.PoseEstimate visionPoseEstimateMT1 = null;
    @Getter private LimelightHelpers.PoseEstimate visionPoseEstimateMT2 = null;
    @Getter private double targetSkewDegrees = 0;
    @Getter private Transform2d poseError = null;
    @Getter private double adjustedSkewAngle = 0.0;
    @Getter boolean rejectUpdate = false;

    @Getter @Setter private boolean applyFilters;



    public Vision(){
        this.cameraList = kCameraList;
        this.applyFilters = kApplyFilters;

        // Vision setup
        // Configure AprilTag Filters 
        /*
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", new int[]{6, 7, 8, 9, 10, 11}); // Only track these tag IDs
        }
        else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", new int[]{17, 18, 19, 20, 21, 22}); // Only track these tag IDs
        }
        else{
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", new int[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}); // Only track these tag IDs
        }
        LimelightHelpers.SetFiducialDownscalingOverride("limelight-right", 4.0f); // Increases the framerate*/

        
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // Permanent
        builder.addBooleanProperty("Use Mega Tag 2", this::isUseMegaTag2, this::setUseMegaTag2);

        builder.addStringProperty("Best Limelight", this::getBestLimeLight, this::setBestLimeLight);

        builder.addBooleanProperty("Robot has tag", this::getRobotHasTag, null);

        builder.addBooleanProperty("Reject Update", this::isRejectUpdate, null);

        
        // Debugging/Testing
        builder.addDoubleProperty("Translational Standard Deviation", this::getTranslationStdDev, this::setTranslationStdDev);
        builder.addDoubleProperty("Rotational Standard Deviation", this::getRotationStdDev, this::setRotationStdDev);
        builder.addDoubleProperty(
            "Vision Pose Estimate Avg Tag Area",
            () -> (visionPoseEstimate != null ? visionPoseEstimate.avgTagArea : 0.0),
            null
        );
        builder.addDoubleProperty(
            "Vision Pose Estimate Ambiguity",
            () -> (visionPoseEstimate != null ? visionPoseEstimate.rawFiducials[0].ambiguity : 0.0),
            null
        );
        builder.addDoubleProperty(
            "Vision Pose Estimate Tag Count",
            () -> (visionPoseEstimate != null ? visionPoseEstimate.tagCount : 0.0),
            null
        );
        builder.addDoubleProperty(
            "Vision Pose Estimate Avg Tag Distance",
            () -> (visionPoseEstimate != null ? visionPoseEstimate.avgTagDist : 0.0),
            null
        );
        builder.addDoubleProperty(
            "Vision Pose Estimate Skew Degrees",
            this::getTargetSkewDegrees,
            null
        );

        builder.addBooleanProperty("LimeLight Right Has Tag", () -> cameraHasTag("limelight-right"), null);
        builder.addBooleanProperty("LimeLight Left Has Tag", () -> cameraHasTag("limelight-left"), null);

        builder.addDoubleProperty("Vision MegaTag1 X Pose Estimate",
        () -> (visionPoseEstimateMT1 != null && visionPoseEstimateMT1.pose != null)
                ? visionPoseEstimateMT1.pose.getX()
                : 0.0,
        null);

        builder.addDoubleProperty("Vision MegaTag1 Y Pose Estimate",
                () -> (visionPoseEstimateMT1 != null && visionPoseEstimateMT1.pose != null)
                        ? visionPoseEstimateMT1.pose.getY()
                        : 0.0,
                null);
        
        builder.addDoubleProperty("Vision MegaTag1 Rotation Pose Estimate",
        () -> (visionPoseEstimateMT1 != null && visionPoseEstimateMT1.pose != null)
                ? visionPoseEstimateMT1.pose.getRotation().getDegrees()
                : 0.0,
        null);
    }


    @Override
    public void periodic(){
        chooseLL();

        // Add vision pose estimate to the pose estimator
        if (!(bestLimeLight.equals(""))){
            updateVisionPoseEstimate();

            if (visionPoseEstimate != null) {
                setStandardDeviation();
                /*
                RobotContainer.getDrivetrain().addVisionMeasurement(visionPoseEstimate.pose, 
                    Utils.fpgaToCurrentTime(visionPoseEstimate.timestampSeconds), 
                    visionStandardDeviation);*/
            }
        }
        

        SmartDashboard.putData(this);
    }

    private void resetBestVisionPoseEstimate(){
        bestLimeLight = ""; // Reset best camera
        visionPoseEstimate =  null;
        visionPoseEstimateMT1 = null;
        visionPoseEstimateMT2 = null;
        targetSkewDegrees = 0;
        adjustedSkewAngle = 0.0;

        poseError = null;
        rejectUpdate = false;

        translationStdDev = 0.0;
        rotationStdDev = kInvalidStandardDeviation;
    }

    /**
     * Chooses the best LimeLight to add to the Pose Estimator
     */
    public void chooseLL(){
        resetBestVisionPoseEstimate();

        double bestTagArea = kMinTagArea; // Start with minimumm acceptable area
        int bestTagCount = 0;
        

        for (String camera : cameraList) {
            boolean hasTag = cameraHasTag(camera);
            LimelightHelpers.SetRobotOrientation(camera, RobotContainer.getDrivetrain().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

            if (hasTag){
                double[] botpose = NetworkTableInstance.getDefault()
                    .getTable(camera)
                    .getEntry("botpose")
                    .getDoubleArray(new double[11]);

                double tagArea = botpose[10];
                SmartDashboard.putNumber("Raw Tag Count", botpose[7]);
                int tagCount = (int) botpose[7];

                if (tagCount > bestTagCount || (tagCount == bestTagCount && tagArea > bestTagArea)){
                    bestLimeLight = camera;
                    bestTagArea = tagArea;
                    bestTagCount = tagCount;
                    visionPoseEstimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimeLight);
                    visionPoseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(bestLimeLight);
                    targetSkewDegrees = LimelightHelpers.getTargetPose_RobotSpace(bestLimeLight)[4];
                    adjustedSkewAngle = 1 / Math.cos(Math.toRadians(targetSkewDegrees)) - 1;
                }
            }
        }
    }


    /**
     * Takes the best limelight and checks the pose estimate against filters before allowing to be sent to the pose estimator
     */
    public void updateVisionPoseEstimate(){
        LimelightHelpers.PoseEstimate visionPoseEstimate = new LimelightHelpers.PoseEstimate();
              
        if (useMegaTag2){
            visionPoseEstimate = visionPoseEstimateMT2;
            // Check to ensure there is a valid pose estimate
            if (visionPoseEstimate == null) {
                rejectUpdate = true;
            }
            else{
                // Reject update if our angular velocity is greater than a threshold degrees per second,
                if (Math.abs(RobotContainer.getDrivetrain().getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > kMaxRotationalRate){
                    rejectUpdate = true;
                }
                // Reject update if there are no visible tags (Redundant?)
                else if (visionPoseEstimate.tagCount == 0) {
                    rejectUpdate = true;
                }
            }  
            
        }
        // Using Megatag1 
        else {
            visionPoseEstimate = visionPoseEstimateMT1;
            poseError = visionPoseEstimate.pose.minus(RobotContainer.getDrivetrain().getState().Pose);
            
            // Check to ensure there is a valid pose estimate
            if (visionPoseEstimate == null || visionPoseEstimate.rawFiducials.length == 0) {
                rejectUpdate = true;
            }   
            else{
                if (Math.abs(RobotContainer.getDrivetrain().getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > kMaxRotationalRate){
                    rejectUpdate = true;
                }
                else if (visionPoseEstimate.tagCount == 0) {
                    rejectUpdate = true;
                }
                else if (Math.hypot(poseError.getX(), poseError.getY()) > kMaxTranslationalErrorMagnitude){
                    rejectUpdate = true;
                }
                else if (visionPoseEstimate.rawFiducials[0].ambiguity > kMaxAmbiguity){
                    rejectUpdate = true;
                }
                else if (Math.abs(poseError.getRotation().getRadians()) > kMaxRotationalErrorMagnitude){
                    rejectUpdate = true;
                }
            }
            
        }

        // Update Pose Estimate if not rejected
        if (rejectUpdate && applyFilters){
            resetBestVisionPoseEstimate();
        }
        else{
            this.visionPoseEstimate =  visionPoseEstimate;
        }
    }

    /**
     * Updated the Translational and Rotational Standard Deviations for the measurement from the best limelight
     */
    public void setStandardDeviation(){
        // April Tag Factors
        if (visionPoseEstimate.avgTagDist > 0.5){
            translationStdDev += 2* 1.2 * Math.sqrt(visionPoseEstimate.avgTagDist - 0.5); 
        }
        
        if (visionPoseEstimate.avgTagDist > 1.25){
            translationStdDev += 2 *1.4 * (adjustedSkewAngle - 1.1) * (adjustedSkewAngle - 1.1);
        }
        
        
        
        if (translationStdDev < kMinimumStandardDeviation){
            translationStdDev = kMinimumStandardDeviation;
        }

        visionStandardDeviation = VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    /**
     * Gets if the specified camera sees an April tag
     * @param camera
     * @return true if there is a visible tag
     */
    public boolean cameraHasTag(String camera){
        return kMinTagArea < NetworkTableInstance.getDefault().getTable(camera).getEntry("botpose").getDoubleArray(new double[11])[10];
    }

    /**
     * Gets if any of the onboard cameras see an April Tag
     * @return true if there is a visible tag
     */
    public boolean getRobotHasTag(){
        for (String camera : cameraList){
            if (cameraHasTag(camera)){
                return true;
            }
        }
        return false;
    }

    /**
     * Gets the tag ID of the best limelight
     * @return tag ID. Returns -1 if there is no tag
     */
    public int getTag(){
        return (int) NetworkTableInstance.getDefault().getTable(bestLimeLight).getEntry("tid").getInteger(-1);
    }

}

/*
 * To Do 
 * add an ambiguity filter back in
 * add field limit filter
 * Calibrate limelights
 */