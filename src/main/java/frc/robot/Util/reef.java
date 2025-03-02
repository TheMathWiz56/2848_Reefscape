package frc.robot.Util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.reef.reefLs;

public class reef {
    private Map<reefLs, Boolean> map = new HashMap<>() {{
        put(reefLs.lL1,true);
        put(reefLs.lL2,true);
        put(reefLs.lL3,true);
        put(reefLs.lL4,true);
        put(reefLs.rL1,true);
        put(reefLs.rL2,true);
        put(reefLs.rL3,true);
        put(reefLs.rL4,true);
    }};

    /** This field type is used in Texas
     */
    public static Map<Integer, Pose2d> tagPoseAndymarkMap = new HashMap<>() {{
        put(6, new Pose2d(13.474, 3.301, new Rotation2d(Math.toRadians(-60))));
        put(7, new Pose2d(13.890, 4.021, new Rotation2d(Math.toRadians(0))));
        put(8, new Pose2d(13.474, 4.740, new Rotation2d(Math.toRadians(60))));
        put(9, new Pose2d(12.643, 4.740, new Rotation2d(Math.toRadians(120))));
        put(10, new Pose2d(12.227, 4.021, new Rotation2d(Math.toRadians(180))));
        put(11, new Pose2d(12.643, 3.301, new Rotation2d(Math.toRadians(-120))));

        put(17, new Pose2d(4.074, 3.301, new Rotation2d(Math.toRadians(-120))));
        put(18, new Pose2d(3.658, 4.021, new Rotation2d(Math.toRadians(180))));
        put(19, new Pose2d(4.074, 4.740, new Rotation2d(Math.toRadians(120))));
        put(20, new Pose2d(4.905, 4.740, new Rotation2d(Math.toRadians(60))));
        put(21, new Pose2d(5.321, 4.021, new Rotation2d(Math.toRadians(0))));
        put(22, new Pose2d(4.905, 3.301, new Rotation2d(Math.toRadians(-60))));
    }};

    /** This field type is used outside of Texas, might be used at State / Worlds
     */
    public Map<Integer, Pose2d> tagPoseWeldedMap = new HashMap<>() {{
        put(6, new Pose2d(13.47415, 3.301, new Rotation2d(Math.toRadians(-60))));
        put(7, new Pose2d(13.89015, 4.021, new Rotation2d(Math.toRadians(0))));
        put(8, new Pose2d(13.47415, 4.74095, new Rotation2d(Math.toRadians(60))));
        put(9, new Pose2d(12.64315, 4.74095, new Rotation2d(Math.toRadians(120))));
        put(10, new Pose2d(12.22715, 4.021, new Rotation2d(Math.toRadians(180))));
        put(11, new Pose2d(12.64315, 3.301, new Rotation2d(Math.toRadians(-120))));

        put(17, new Pose2d(4.07415, 3.301, new Rotation2d(Math.toRadians(-120))));
        put(18, new Pose2d(3.65815, 4.021, new Rotation2d(Math.toRadians(180))));
        put(19, new Pose2d(4.07415, 4.74095, new Rotation2d(Math.toRadians(120))));
        put(20, new Pose2d(4.90515, 4.74095, new Rotation2d(Math.toRadians(60))));
        put(21, new Pose2d(5.32115, 4.021, new Rotation2d(Math.toRadians(0))));
        put(22, new Pose2d(4.90515, 3.301, new Rotation2d(Math.toRadians(-60))));
    }};

    /*update any reef value */
    public void update(reefLs pos,boolean val){
        map.put(pos,val);
    }
    public boolean get(reefLs pos){
        return map.get(pos);
    }
}


