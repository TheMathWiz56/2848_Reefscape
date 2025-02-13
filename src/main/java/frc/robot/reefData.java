package frc.robot;

import java.util.List;

import frc.robot.utils.reef;;

public class reefData {
    public static reef r1 = new reef();
    public static reef r2 = new reef();
    public static reef r3 = new reef();
    public static reef r4 = new reef();
    public static reef r5 = new reef();
    public static reef r6 = new reef();
    private static List<reef> reefs = List.of(r1,r2,r3,r4,r5,r6);
    public void reset(){
        r1 = new reef();
        r2 = new reef();
        r3 = new reef();
        r4 = new reef();
        r5 = new reef();
        r6 = new reef();
        }
    
    public static void update(int id,constants.reefData.reef loc, boolean val){
        reefs.get(id).update(loc,val);
    }
}
