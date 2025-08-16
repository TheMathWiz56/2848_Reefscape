package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.NetworkTableInstance;

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
    private String bestLimeLight = "";
    @Getter private double translationStdDev = kInvalidStandardDeviation;
    @Getter private double translationRotDev;
    @Getter private LimelightHelpers.PoseEstimate visionPoseEstimate = null;

    public Vision(){
        this.cameraList = kCameraList;
        
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        
    }


    @Override
    public void periodic(){
        
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
     * Updated the Translational and Rotational Standard Deviations for the measurement from the best limelight
     */
    public void setStandardDeviation(){

    }

    public void updateVisionPoseEstimate(){
        boolean rejectUpdate = false;
        LimelightHelpers.PoseEstimate visionPoseEstimate = new LimelightHelpers.PoseEstimate();

        for (String camera : cameraList){
            LimelightHelpers.SetRobotOrientation(camera, RobotContainer.getDrivetrain().getState().Pose.getRotation().getDegrees(),
        0, 0, 0, 0, 0);
        }


        if (useMegaTag2 == false) {
            visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimeLight);

            if (visionPoseEstimate == null){
                rejectUpdate = true;
            }
            else{
                if (visionPoseEstimate.tagCount == 1 && visionPoseEstimate.rawFiducials.length == 1) {
                    if (visionPoseEstimate.rawFiducials[0].ambiguity > .7) {
                        rejectUpdate = true;
                    }
                    if (visionPoseEstimate.rawFiducials[0].distToCamera > 3) {
                        rejectUpdate = true;
                    }
                    }
                    if (visionPoseEstimate.tagCount == 0) {
                    rejectUpdate = true;
                    }
            }
        }
        else{
            visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(bestLimeLight);
            if (visionPoseEstimate == null) {
                rejectUpdate = true;
            } 
            else {
                if (Math.abs(RobotContainer.getDrivetrain().getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720){ // if our angular velocity is greater than 720 degrees per second,
                    rejectUpdate = true;
                }
                if (visionPoseEstimate.tagCount == 0) {
                    rejectUpdate = true;
                }
            }
        }

        if (rejectUpdate){
            this.visionPoseEstimate =  null;
        }
        else{
            this.visionPoseEstimate =  visionPoseEstimate;
        }
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


/*
 * Choose LL
 * Get Tag
 * LLHas Tag
 * UseMegaTag
 */