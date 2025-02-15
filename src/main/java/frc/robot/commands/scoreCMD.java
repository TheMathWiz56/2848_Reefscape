package frc.robot.commands;



import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
public class scoreCMD extends Command {
  
  private final Elevator m_subsystem;

  public scoreCMD(Elevator subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
