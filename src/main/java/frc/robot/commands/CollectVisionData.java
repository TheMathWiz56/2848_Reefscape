package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Util.reef;

public class CollectVisionData {
    private final CommandSwerveDrivetrain drive;
    private final Translation2d[] positionArray = {new Translation2d(5.821, 4.021), 
        new Translation2d(6.321, 4.021), new Translation2d(6.821, 4.021), 
        new Translation2d(7.321, 4.021)};

    public CollectVisionData(CommandSwerveDrivetrain drive){
        this.drive = drive;
    }

    public Command collectVisionData(){
        Command[] commandArray = new Command[positionArray.length * 2];

        for (int i = 0; i < positionArray.length; i++) {
            Translation2d position = positionArray[i];
            commandArray[i * 2] = drive.testPathPIDTo(new Pose2d(position, new Rotation2d()), reef.tagPoseAndymarkMap.get(21));
            commandArray[i * 2 + 1] = new WaitCommand(2.0); // seconds
        }

        return new SequentialCommandGroup(commandArray);
    }
}
