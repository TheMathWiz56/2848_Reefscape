package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Util.reef;

public class CollectVisionData {
    private final CommandSwerveDrivetrain drive;
    private final Translation2d[] positionArray = {new Translation2d(6.321, 4.021), 
        new Translation2d(6.821, 4.021), 
        new Translation2d(7.321, 4.021), new Translation2d(7.821, 4.021)};

    private final double[] angleArray = {-60,45,-30,15,-15,30,-45,60};

    public CollectVisionData(CommandSwerveDrivetrain drive){
        this.drive = drive;
    }

    /**
     * Creates a drivebase command that drives to a triangular set of coordinates and sweeps the camera to collect vision pose estimate data
     * @return Sequential Command Group
     */
    public Command collectVisionData(){
        List<Command> commands = new ArrayList<>();

        Pose2d tagpose = reef.tagPoseAndymarkMap.get(21);

        // Sweep +- 60 degrees from the target
        for (Translation2d position : positionArray){
            commands.add(drive.testPathPIDTo(new Pose2d(position, tagpose.getRotation().rotateBy(new Rotation2d(Math.PI))), tagpose));
            commands.add(drive.stop());
            commands.add(new WaitCommand(1));

            for (double angle : angleArray){
                commands.add(drive.testPathPIDTo(new Pose2d(position, tagpose.getRotation().rotateBy(new Rotation2d(Math.PI + Math.toRadians(angle)))), tagpose));
                commands.add(drive.stop());
                commands.add(new WaitCommand(1));
            }

        }

        return new SequentialCommandGroup(commands.toArray(new Command[0]));
    }

    public Command testCollectVisionData(){
        List<Command> commands = new ArrayList<>();

        Pose2d tagpose = reef.tagPoseAndymarkMap.get(21);

        Translation2d position = positionArray[3];

        commands.add(drive.testPathPIDTo(new Pose2d(position, tagpose.getRotation().rotateBy(new Rotation2d(Math.PI))), tagpose));
        Command command1 = drive.testPathPIDTo(new Pose2d(position, tagpose.getRotation().rotateBy(new Rotation2d(Math.PI))), tagpose);
        //commands.add(drive.stop());
        commands.add(new WaitCommand(1));

        commands.add(drive.testPathPIDTo(new Pose2d(position, tagpose.getRotation().rotateBy(new Rotation2d(Math.PI + Math.toRadians(60)))), tagpose));
        Command command2 = drive.testPathPIDTo(new Pose2d(position, tagpose.getRotation().rotateBy(new Rotation2d(Math.PI + Math.toRadians(60)))), tagpose);
        //commands.add(drive.stop());
        commands.add(new WaitCommand(1));

        commands.add(drive.testPathPIDTo(new Pose2d(position, tagpose.getRotation().rotateBy(new Rotation2d(Math.PI + Math.toRadians(-60)))), tagpose));
        Command command3 = drive.testPathPIDTo(new Pose2d(position, tagpose.getRotation().rotateBy(new Rotation2d(Math.PI + Math.toRadians(-60)))), tagpose);
        //commands.add(drive.stop());
        commands.add(new WaitCommand(1));

        return command1.andThen(new WaitCommand(1)).andThen(command2).andThen(new WaitCommand(1)).andThen(command3).andThen(new WaitCommand(1));
    }
}
