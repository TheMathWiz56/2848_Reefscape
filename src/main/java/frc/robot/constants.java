package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;

public class constants {
    //data about april tags
    public static final class tagData{
        //tags visible from red alliance
        public static final class red{
            //all possible reef tags
            public static final List<Integer> reef = List.of(6,7,8,9,10,11);
            //tags facing red alliance on the barge
            public static final List<Integer> barge = List.of(4,5);
            //tags at human player areas
            public static final List<Integer> human = List.of(1,2);
            //processor tag
            public static final int processor = 3;
        }
        //tags visible from blue alliance
        public static final class blue{
            //all possible reef tags
            public static final List<Integer> reef = List.of(17,18,19,20,21,22);
            //tags facing red alliance on the barge
            public static final List<Integer> barge = List.of(14,15);
            //tags at human player areas
            public static final List<Integer> human = List.of(12,13);
            //processor tag
            public static final int processor = 16;
        }
    }
    public static final class reefData{
        //all possible reefs
        public static enum reef{
            Rl4,
            Rl3,
            Rl2,
            Rl1,
            Ll4,
            Ll3,
            Ll2,
            Ll1
        }
    //Filled in with temp values for now
    //[offset from tag,elevator height]
    public static Map<reef, Double[]> scoreData = new HashMap<>() {{
    put(reef.Rl4,new Double[]{-6.0,4000.0});
    put(reef.Rl3,new Double[]{-6.0,3000.0});
    put(reef.Rl2,new Double[]{-6.0,2000.0});
    put(reef.Rl1,new Double[]{-6.0,1000.0});
    put(reef.Ll4,new Double[]{6.0,4000.0});
    put(reef.Ll3,new Double[]{6.0,3000.0});
    put(reef.Ll2,new Double[]{6.0,2000.0});
    put(reef.Ll1,new Double[]{6.0,2000.0});
        }};
    public static Map<reef,Pose2d> posData = new HashMap<>(){{

    }};    
    }
    public static final class elevatorConstants{
        public static final double kS = 5;
        public static final double kG = 5;
        public static final double kV = 5;
        public static final double kA = 5;

    }
}
