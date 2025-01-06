package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class tempMoveCMD extends Command{
    CommandSwerveDrivetrain m_drivetrain;

    public tempMoveCMD(CommandSwerveDrivetrain drivetrain){
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }
    @Override
    public void initialize(){
        
    }

}
