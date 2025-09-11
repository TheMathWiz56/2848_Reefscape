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
    @Getter @Setter private boolean addToPoseEstimator;
    @Getter @Setter private boolean useOldStdDev = true;


    public Vision(){
        this.cameraList = kCameraList;
        this.applyFilters = kApplyFilters;
        this.addToPoseEstimator = kAddToPoseEstimator;
        configureCameras();
    }

    public void configureCameras(){
        for (String camera : cameraList){
            configureCamera(camera);
        }
    }

    private void configureCamera(String camera){
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            LimelightHelpers.SetFiducialIDFiltersOverride(camera, kRedAprilTagList); // Only track these tag IDs
        }
        else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            LimelightHelpers.SetFiducialIDFiltersOverride(camera, kBlueAprilTagList); // Only track these tag IDs
        }
        else{
            LimelightHelpers.SetFiducialIDFiltersOverride(camera, kAllAprilTagList); // Only track these tag IDs
        }
        LimelightHelpers.SetFiducialDownscalingOverride(camera, kDownscaleFactor); // Increases the framerate

        // Force LEDs off
        LimelightHelpers.setLEDMode_ForceOff(camera);

        // Apply window crop settings to increase framerate
        CropWindowSettings cropWindow = cameraCropWindowMap.get(camera);
        LimelightHelpers.setCropWindow(camera, cropWindow.getCropXMin(), cropWindow.getCropXMax(), cropWindow.getCropYMin(), cropWindow.getCropYMax());
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // Permanent
        builder.addStringProperty("Best Limelight", this::getBestLimeLight, this::setBestLimeLight);

        builder.addBooleanProperty("Is Using Mega Tag 2", this::isUseMegaTag2, this::setUseMegaTag2);
        builder.addBooleanProperty("Robot has tag", this::getRobotHasTag, null);
        builder.addBooleanProperty("Is Rejecting Update", this::isRejectUpdate, null);
        builder.addBooleanProperty("Is Applying Filters", this::isApplyFilters, this::setApplyFilters);
        builder.addBooleanProperty("Is Adding To Pose Estimator", this::isAddToPoseEstimator, this::setAddToPoseEstimator);

        builder.addBooleanProperty("Use Old Std Dev", this::isUseOldStdDev, this::setUseOldStdDev);

        
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

                if(addToPoseEstimator){
                    RobotContainer.getDrivetrain().addVisionMeasurement(visionPoseEstimate.pose, 
                        Utils.fpgaToCurrentTime(visionPoseEstimate.timestampSeconds), 
                        visionStandardDeviation);
                }
            }
        }
        

        SmartDashboard.putData(this);
    }

    /**
     * Resets all stored variables for best camera and pose estimate readings
     */
    private void resetBestVisionPoseEstimate(){
        bestLimeLight = "";
        visionPoseEstimate =  null;
        visionPoseEstimateMT1 = null;
        visionPoseEstimateMT2 = null;
        targetSkewDegrees = 0;
        adjustedSkewAngle = 0.0;

        poseError = null;
        rejectUpdate = false;

        translationStdDev = kInvalidStandardDeviation;
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
            // Sets the robot orientation to each camera, used for MegaTag2
            LimelightHelpers.SetRobotOrientation(camera, RobotContainer.getDrivetrain().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

            if (hasTag){
                double[] botpose = NetworkTableInstance.getDefault()
                    .getTable(camera)
                    .getEntry("botpose")
                    .getDoubleArray(new double[11]);

                double tagArea = botpose[10];
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
        }
        else {
            // Using Megatag1 
            visionPoseEstimate = visionPoseEstimateMT1;
            poseError = visionPoseEstimate.pose.minus(RobotContainer.getDrivetrain().getState().Pose);
            
            // Check to ensure there is a valid pose estimate
            if (visionPoseEstimate == null || visionPoseEstimate.rawFiducials.length == 0) {
                rejectUpdate = true;
            }
            // Reject update if our angular velocity is greater than a threshold degrees per second
            else if (Math.abs(RobotContainer.getDrivetrain().getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > kMaxRotationalRate){
                rejectUpdate = true;
            }
            // Reject update if there are no visible tags (Redundant?)
            else if (visionPoseEstimate.tagCount == 0) {
                rejectUpdate = true;
            }
            // Reject update if the translational error magnitude is larger than the threshold
            // else if (Math.hypot(poseError.getX(), poseError.getY()) > kMaxTranslationalErrorMagnitude){
            //     rejectUpdate = true;
            // }
            // Reject update if the ambiguity is larger than the threshold
            else if (visionPoseEstimate.rawFiducials[0].ambiguity > kMaxAmbiguity){
                rejectUpdate = true;
            }
            // Reject update if the rotational error magnitude is larger than the threshold
            // else if (Math.abs(poseError.getRotation().getRadians()) > kMaxRotationalErrorMagnitude){
            //     rejectUpdate = true;
            // }
            // Reject update if the pose estimate is not inside of the field
            else if (visionPoseEstimate.pose.getX() < 0.0 || visionPoseEstimate.pose.getY() < 0.0 || 
                        visionPoseEstimate.pose.getX() > kFieldLength || visionPoseEstimate.pose.getY() > kFieldWidth){
                rejectUpdate = true;
            }
        }

        // Update Pose Estimate if not rejected and is applying filters
        if (rejectUpdate && applyFilters){
            visionPoseEstimate =  null;
        }
        else{
            this.visionPoseEstimate =  visionPoseEstimate;
        }
    }

    /**
     * Updated the Translational and Rotational Standard Deviations for the measurement from the best limelight
     */
    public void setStandardDeviation(){
        if (visionPoseEstimate.tagCount > 1){
            // If the camera sees more than 1 april tag assume the lowest standard deviation
            translationStdDev = kMinimumTranslationalStandardDeviation;
            rotationStdDev = kMinimumRotationalStandardDeviation;
        }
        else if (useOldStdDev){
            translationStdDev = (visionPoseEstimate.avgTagArea * (-18.3)) + 11.34;
            rotationStdDev = 0.3 * visionPoseEstimate.avgTagDist - 0.1;

            // Modify standard deviations using tag skew data if past the distance threshold
            if (visionPoseEstimate.avgTagDist > kAddSkewDataDistanceThreshold){
                rotationStdDev += 0.02 * (adjustedSkewAngle - 4.1) * (adjustedSkewAngle - 4.1);
            }
        }
        else{
            // Set standard deviations using avg tag distance data
            //limited to more than 0.5 because of the sqrt function
            if (visionPoseEstimate.avgTagDist > 0.5){
                translationStdDev = 0.25 * 1.2 * Math.sqrt(visionPoseEstimate.avgTagDist - 0.5);
                rotationStdDev = 0.3 * visionPoseEstimate.avgTagDist - 0.1;
            }
            
            // Modify standard deviations using tag skew data if past the distance threshold
            if (visionPoseEstimate.avgTagDist > kAddSkewDataDistanceThreshold){
                translationStdDev += 1.4 * (adjustedSkewAngle - 1.1) * (adjustedSkewAngle - 1.1);
                rotationStdDev += 0.02 * (adjustedSkewAngle - 4.1) * (adjustedSkewAngle - 4.1);
            }
        }

        // Check to make sure standard deviations are not below the minimum limits
        if (translationStdDev < kMinimumTranslationalStandardDeviation){
            translationStdDev = kMinimumTranslationalStandardDeviation;
        }
        if (rotationStdDev < kMinimumRotationalStandardDeviation){
            rotationStdDev = kMinimumRotationalStandardDeviation;
        }

        SmartDashboard.putNumber("Old Standard Deviation", (visionPoseEstimate.avgTagArea * (-18.3)) + 11.34);

        // Fill out Standard deviation matrix for drivebase
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
 * Calibrate limelights
 * Conditions for filters and std devs when there are two visible tags
 * figure out window crop values to increase framerate?
 * but the limelight 3G lenses
 * pathPID include initial velocity
 * don't allow auto align score algae until corrected with the limelights
 * marge back into main
 */