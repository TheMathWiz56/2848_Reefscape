package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import java.util.ArrayList;
import java.util.List;

public class keypad{
    public static List<Integer> keys = new ArrayList<>();
    public static keyMode mode = keyMode.SCORE;

    public static enum keyMode{
        SCORE,
        MARK
    }
    private CommandGenericHID operatorKeypad= new CommandGenericHID(1);;

    public keypad(){
        
    }


    
    public void update(){
        keys.clear();
        if(operatorKeypad.button(15).getAsBoolean()){
            if(mode==keyMode.SCORE){
                mode = keyMode.MARK;
            } else{
                mode = keyMode.SCORE;
            }
        }
        for(int i =1; i <24;i++){
            if(operatorKeypad.button(i).getAsBoolean()){
                keys.add(i);
            }
        }
    }




}
