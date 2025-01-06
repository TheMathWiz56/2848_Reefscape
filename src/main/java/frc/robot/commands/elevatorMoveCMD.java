package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.elevator;
public class elevatorMoveCMD extends Command{
    private constants.reefData.reef reef;
    private elevator ele;
    public elevatorMoveCMD(elevator Elevator,constants.reefData.reef reef){
        this.reef = reef;
        this.ele = Elevator;
        addRequirements(ele);
    }
    @Override
    public void initialize(){
        
    }
    
}
