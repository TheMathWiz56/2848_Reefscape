package frc.robot;

import java.util.List;

import frc.robot.Util.reef;
import java.util.Map;
import java.util.HashMap;

public class reefData {
    private static reef r1 = new reef();
    private static reef r2 = new reef();
    private static reef r3 = new reef();
    private static reef r4 = new reef();
    private static reef r5 = new reef();
    private static reef r6 = new reef();
    //private static List<reef> reefs = List.of(r1,r2,r3,r4,r5,r6);
    private static Map<Integer,reef>  reefs = new HashMap<>() {{
        put(1, r1);
        put(2, r2);
        put(3, r3);
        put(4, r4);
        put(5, r5);
        put(6, r6);
    }};
    public static void reset(){
        r1 = new reef();
        r2 = new reef();
        r3 = new reef();
        r4 = new reef();
        r5 = new reef();
        r6 = new reef();
        reefs.put(1, r1);
        reefs.put(2, r2);
        reefs.put(3, r3);
        reefs.put(4, r4);
        reefs.put(5, r5);
        reefs.put(6, r6);
        }
        /*update reef, reef id(1-6), which L, set value */
    public static void update(int id,Constants.reef.reefLs loc, boolean val){
        reefs.get(id).update(loc,val);
    }
    public static boolean open(int id, Constants.reef.reefLs loc){
        return reefs.get(id).get(loc);
    }
}
