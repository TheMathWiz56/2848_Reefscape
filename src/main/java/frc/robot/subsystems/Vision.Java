package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import lombok.Getter;

public class Vision extends SubsystemBase {

    private boolean useMegaTag2 = false;
    private String[] cameraList;

    public Vision(String[] cameraList){

        
    }

    @Override
    public void periodic(){
        RobotContainer.getDrivetrain().
    }

    

}


/*
 * Choose LL
 * Get Tag
 * LLHas Tag
 * UseMegaTag
 */