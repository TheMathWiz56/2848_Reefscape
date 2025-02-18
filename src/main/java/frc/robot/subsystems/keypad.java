package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants;
import frc.robot.commands.scoreCMD;

import java.util.ArrayList;
import java.util.List;

public class keypad {
    public static List<Integer> keys = new ArrayList<>();
    public static keyMode mode = keyMode.SCORE;
    public static int reef = 0;
    public static Constants.reef.reefLs L = Constants.reef.reefLs.NONE;

    public static enum keyMode {
        SCORE,
        MARK
    }

    private CommandGenericHID operatorKeypad = new CommandGenericHID(1);;

    public keypad() {

    }

    public void update() {
        keys.clear();
        if (operatorKeypad.button(15).getAsBoolean()) {
            if (mode == keyMode.SCORE) {
                mode = keyMode.MARK;
            } else {
                mode = keyMode.SCORE;
            }
        }
        for (int i = 1; i < 24; i++) {
            if (operatorKeypad.button(i).getAsBoolean()) {
                keys.add(i);
            }
        }
        for (int i : keys) {
            if (Constants.reef.rMap.containsKey(i)) {
                reef = Constants.reef.rMap.get(i);
            }else{
                reef =0;
            }
            if (Constants.reef.lMap.containsKey(i)) {
                L = Constants.reef.lMap.get(i);
            } else{
                L = Constants.reef.reefLs.NONE;
            }
        }

        
    }

}
