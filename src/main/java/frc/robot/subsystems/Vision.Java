package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
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

    public Vision(){
        this.cameraList = kCameraList;
        
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // Permanent
        builder.addBooleanProperty("Use Mega Tag 2", this::isUseMegaTag2, this::setUseMegaTag2);

        builder.addStringProperty("Best Limelight", this::getBestLimeLight, this::setBestLimeLight);

        /*
        // Debugging/Testing
        builder.addDoubleProperty("Translational Standard Deviation", this::getTranslationStdDev, this::setTranslationStdDev);
        builder.addDoubleProperty("Rotational Standard Deviation", this::getRotationStdDev, this::setRotationStdDev);
        builder.addDoubleProperty(
            "Pose Estimate Avg Tag Area",
            () -> (visionPoseEstimate != null ? visionPoseEstimate.avgTagArea : 0.0),
            null
        );
        builder.addDoubleProperty(
            "Pose Estimate Ambiguity",
            () -> (visionPoseEstimate != null ? visionPoseEstimate.rawFiducials[0].ambiguity : 0.0),
            null
        );
        builder.addDoubleProperty(
            "Pose Estimate Tag Count",
            () -> (visionPoseEstimate != null ? visionPoseEstimate.tagCount : 0.0),
            null
        );
        */
    }


    @Override
    public void periodic(){
        chooseLL();

        // Add vision pose estimate to the pose estimator
        if (!bestLimeLight.equals("") && visionPoseEstimate != null) {
            updateVisionPoseEstimate();
            setStandardDeviation();
            RobotContainer.getDrivetrain().addVisionMeasurement(visionPoseEstimate.pose, 
                Utils.fpgaToCurrentTime(visionPoseEstimate.timestampSeconds), 
                visionStandardDeviation);
        }

        SmartDashboard.putData(this);
    }

    /**
     * Chooses the best LimeLight to add to the Pose Estimator
     */
    public void chooseLL(){
        bestLimeLight = ""; // Reset best camera
        double bestTagArea = kMinAcceptableTagArea; // Start with minimumm acceptable area

        for (String camera : cameraList) {
            double tagArea = NetworkTableInstance.getDefault()
                                .getTable(camera)
                                .getEntry("botpose")
                                .getDoubleArray(new double[11])[10];
            boolean hasTag = cameraHasTag(camera);

            if (hasTag && tagArea > bestTagArea) {
                bestLimeLight = camera;
                bestTagArea = tagArea;
            }
        }
    }


    /**
     * Takes the best limelight and checks the pose estimate against filters before allowing to be sent to the pose estimator
     */
    public void updateVisionPoseEstimate(){
        boolean rejectUpdate = false;
        LimelightHelpers.PoseEstimate visionPoseEstimate = new LimelightHelpers.PoseEstimate();
              
        if (useMegaTag2){
            visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(bestLimeLight);
            // Check to ensure there is a valid pose estimate
            if (visionPoseEstimate == null) {
                rejectUpdate = true;
            }   

            // Update robot orientation for MegaTag2
            for (String camera : cameraList){
                LimelightHelpers.SetRobotOrientation(camera, RobotContainer.getDrivetrain().getState().Pose.getRotation().getDegrees(),
            0, 0, 0, 0, 0);
            }

            // Reject update if our angular velocity is greater than a threshold degrees per second,
            if (Math.abs(RobotContainer.getDrivetrain().getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > kMaxAcceptableRotationalRate){
                rejectUpdate = true;
            }
            // Reject update if there are no visible tags (Redundant?)
            else if (visionPoseEstimate.tagCount == 0) {
                rejectUpdate = true;
            }
        }
        // Using Megatag1 
        else {
            visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimeLight);
            // Check to ensure there is a valid pose estimate
            if (visionPoseEstimate == null) {
                rejectUpdate = true;
            }   

            // Reject update if our angular velocity is greater than a threshold degrees per second,
            if (Math.abs(RobotContainer.getDrivetrain().getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > kMaxAcceptableRotationalRate){
                rejectUpdate = true;
            }
            else if (visionPoseEstimate.rawFiducials[0].ambiguity > kMaxAcceptableAmbiguity) {
                rejectUpdate = true;
            }
            else if (visionPoseEstimate.rawFiducials[0].distToCamera > kMaxAcceptableDistance) {
                rejectUpdate = true;
            }
            else if (visionPoseEstimate.tagCount == 0) {
            rejectUpdate = true;
            }
        }

        // Update Pose Estimate if not rejected
        if (rejectUpdate){
            this.visionPoseEstimate =  null;
        }
        else{
            this.visionPoseEstimate =  visionPoseEstimate;
        }
    }

    /**
     * Updated the Translational and Rotational Standard Deviations for the measurement from the best limelight
     */
    public void setStandardDeviation(){
        translationStdDev = 0.0;
        // April Tag Factors
        translationStdDev += kAreaMultiplier * visionPoseEstimate.avgTagArea + kAreaConstant;
        translationStdDev += kAmbiguityMultiplier * visionPoseEstimate.rawFiducials[0].ambiguity + kAmbiguityConstant; // only accounts for the ambiguity of 1 visible tag
        translationStdDev += kTagCountMultiplier * visionPoseEstimate.tagCount + kTagCountConstant;

        // Velocity Factors
        translationStdDev += kAngularVelocityMultiplier * 
            Math.hypot(RobotContainer.getDrivetrain().getState().Speeds.vxMetersPerSecond, RobotContainer.getDrivetrain().getState().Speeds.vyMetersPerSecond) + 
            kAngularVelocityConstant;
        translationStdDev += kLinearVelocityMultiplier * 
            RobotContainer.getDrivetrain().getState().Speeds.vxMetersPerSecond + kLinearVelocityConstant;

        visionStandardDeviation = VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    /**
     * Gets if the specified camera sees an April tag
     * @param camera
     * @return true if there is a visible tag
     */
    public boolean cameraHasTag(String camera){
        return kMinAcceptableTagArea < NetworkTableInstance.getDefault().getTable(camera).getEntry("botpose").getDoubleArray(new double[11])[10];
    }

    /**
     * Gets if any of the onboard cameras see an April Tag
     * @return true if there is a visible tag
     */
    public boolean robotHasTag(){
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