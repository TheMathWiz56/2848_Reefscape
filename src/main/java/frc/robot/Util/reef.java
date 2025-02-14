package frc.robot.Util;

import java.util.HashMap;
import java.util.Map;

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
/*update any reef value */
public void update(reefLs pos,boolean val){
    map.put(pos,val);
}
public boolean get(reefLs pos){
    return map.get(pos);
}
}


