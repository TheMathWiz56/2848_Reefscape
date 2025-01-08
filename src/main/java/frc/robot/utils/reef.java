package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import frc.robot.constants.reefData;

public class reef {
    private Map<reefData.reef, Boolean> map = new HashMap<>() {{
        put(reefData.reef.Rl4,true);
        put(reefData.reef.Rl3,true);
        put(reefData.reef.Rl2,true);
        put(reefData.reef.Rl1,true);
        put(reefData.reef.Ll4,true);
        put(reefData.reef.Ll3,true);
        put(reefData.reef.Ll2,true);
        put(reefData.reef.Ll1,true);
}};
public void update(reefData.reef pos,boolean val){
    map.put(pos,val);
}
}


