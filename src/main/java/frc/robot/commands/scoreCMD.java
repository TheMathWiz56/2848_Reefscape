package frc.robot.commands;



import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.reef.reefSide;
import frc.robot.Constants.reef.reefLs;
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
public class scoreCMD extends Command {
  
  private final Elevator m_elevator;
  
  private final reefLs L;

  public scoreCMD(Elevator elevator,reefLs L) {
    m_elevator = elevator;
    this.L = L;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
