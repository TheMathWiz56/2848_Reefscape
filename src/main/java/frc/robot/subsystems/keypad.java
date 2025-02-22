package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;


import java.util.ArrayList;
import java.util.List;

public class keypad extends SubsystemBase{
    public static List<Integer> keys = new ArrayList<>();
    public static keyMode mode = keyMode.SCORE;
    public static int reef = 0;
    public static Constants.reef.reefLs L = Constants.reef.reefLs.NONE;
    private static Constants.robotStates.intakeStates intakeState = Constants.robotStates.intakeStates.STOP;

    public static enum keyMode {
        SCORE,
        MARK
    }

    private CommandGenericHID operatorKeypad = new CommandGenericHID(1);

    public keypad() {
        operatorKeypad = new CommandGenericHID(1);
    }
        @Override
    public void periodic() {
        SmartDashboard.putData(this);
        
        keys.clear();
        if (operatorKeypad.button(15).getAsBoolean()) {
            if (mode == keyMode.SCORE) {
                mode = keyMode.MARK;
            } else {
                mode = keyMode.SCORE;
            }
        }
        for (int i = 1; i < 28; i++) {
            if (operatorKeypad.button(i).getAsBoolean()) {
                keys.add(i);
            }
        }

        boolean reefSet = false;
        boolean lSet = false;
        
        for (int i : keys) {
            if (Constants.reef.rMap.containsKey(i) ) {
                reef = Constants.reef.rMap.get(i);
            }else if(!Constants.reef.lMap.containsKey(i)){
                reef =0;
            }
            if (Constants.reef.lMap.containsKey(i)) {
                L = Constants.reef.lMap.get(i);
                
            } else if(!Constants.reef.rMap.containsKey(i)){
                L = Constants.reef.reefLs.NONE;
            }

            if(i==20){
                intakeState = Constants.robotStates.intakeStates.INTAKE;
            }
            if(i== 21){
                intakeState = Constants.robotStates.intakeStates.STOP;
            }
            if(i==28){
                intakeState = Constants.robotStates.intakeStates.EXHAUST;
            }
        }
        if(keys.size()==0){
            reef = 0;
            L = Constants.reef.reefLs.NONE;
        }

        
    }

    
    public int getKeysSize(){
        return keys.size();
    }

    public int getReef(){
        return reef;
    }


    public Constants.reef.reefLs getReefL(){
        return L;
    }
    public boolean scoreReef(){
        return reef!=0 && mode == keyMode.SCORE && L != Constants.reef.reefLs.NONE;
    }
    public boolean feed(){
        return keys.contains(20);
    }
    public boolean stow(){
        return keys.contains(22);
    }
    public boolean intakeStart(){
        return keys.contains(20);
    }
    public boolean intakeStop(){
        return keys.contains(21);
    }
    public boolean intakeExhaust(){
        return keys.contains(28);
    }
    public boolean cancelScore(){
        return keys.contains(19);
    }
    public boolean net(){
        return keys.contains(27);
    }
    public boolean climb(){
        return keys.contains(23);
    }
    public boolean climbCancel(){
        return keys.contains(24);
    }
    public boolean reefAlgaeHigh(){
        return keys.contains(25);
    }
    public boolean reefAlgaeLow(){
        return keys.contains(26);
    }
    
    public void initSendable(SendableBuilder builder) {
        builder.addIntegerProperty("Reef selected", () -> reef, null);
        builder.addIntegerProperty("keydown size", () -> keys.size(), null);
        builder.addStringProperty("key presses", () -> keys.toString(), null);
    }

}
