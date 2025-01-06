package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.elevatorConstants.*;
import frc.robot.constants;

public class elevator extends SubsystemBase {
    private Spark eMotor = new Spark(0);
    private Encoder encoder = new Encoder(0,1);
    private ElevatorFeedforward controller = new ElevatorFeedforward(kS,kG,kV,kA);


    public void setSetPoint(constants.reefData.reef reef){
    }

    @Override
    public void periodic(){
        encoder.getRate();
        controller.calculate(encoder.getRate());
    }
}
