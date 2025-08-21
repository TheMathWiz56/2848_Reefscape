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

    private static ArrayList<Translation2d> positionArray = new ArrayList<>();

    private final double[] angleArray = {-60,-45,-30,-15,15,30,45,60};

    public CollectVisionData(CommandSwerveDrivetrain drive){
        this.drive = drive;

        // Row 2
        /*
        positionArray.add(new Translation2d(7.321, 5.521));

        // Row 3
        positionArray.add(new Translation2d(6.821, 5.021));
        positionArray.add(new Translation2d(7.821, 5.021));

        // Row 4
        positionArray.add(new Translation2d(6.312, 4.521));
        positionArray.add(new Translation2d(7.321, 4.521));
        positionArray.add(new Translation2d(7.821, 4.521));

        // Row 5
        positionArray.add(new Translation2d(7.321, 4.021));
        */
        
        positionArray.add(new Translation2d(7.821, 4.021));

        // Row 6
        positionArray.add(new Translation2d(6.312, 3.521));
        positionArray.add(new Translation2d(7.321, 3.521));
        positionArray.add(new Translation2d(7.821, 3.521));

        // Row 7
        positionArray.add(new Translation2d(6.821, 3.021));
        positionArray.add(new Translation2d(7.821, 3.021));

        // Row 8
        positionArray.add(new Translation2d(7.321, 2.521));
    }

    /**
     * Creates a drivebase command that drives to a triangular set of coordinates and sweeps the camera to collect vision pose estimate data
     * Decreased PathPID acceleration limits and only allow 1 tag 
     * @return Sequential Command Group
     */
    public Command collectVisionData(){
        List<Command> commands = new ArrayList<>();

        Pose2d tagpose = reef.tagPoseAndymarkMap.get(21);

        // Sweep +- 60 degrees from the target
        for (Translation2d position : positionArray){
            commands.add(drive.testPathPIDTo(new Pose2d(position, tagpose.getRotation().rotateBy(new Rotation2d(Math.PI))), tagpose));
            commands.add(drive.stop());
            commands.add(new WaitCommand(0.3));

            for (double angle : angleArray){

                commands.add(drive.testPathPIDTo(new Pose2d(position, tagpose.getRotation().rotateBy(new Rotation2d(Math.PI + Math.toRadians(angle)))), tagpose));
                commands.add(drive.stop());
                commands.add(new WaitCommand(0.3));

            }

        }

        return new SequentialCommandGroup(commands.toArray(new Command[0]));
    }
}
