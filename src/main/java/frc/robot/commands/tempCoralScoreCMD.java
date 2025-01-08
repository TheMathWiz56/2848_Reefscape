package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.reefData;
public class tempCoralScoreCMD extends Command {
    private constants.reefData.reef goalReef;
    private int id;
    public tempCoralScoreCMD(constants.reefData.reef reef,int reefId){
        goalReef = reef;
        id = reefId;
    }

    @Override
    public void initialize(){

    }
    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            reefData.update(id,goalReef,true);
        } else{
            reefData.update(id,goalReef,false);
        }
    }
}
