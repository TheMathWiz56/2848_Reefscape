package frc.robot.subsystems;

import static frc.robot.Constants.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
    private final AddressableLED led = new AddressableLED(kPwmPort);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(kNumberOfLEDs);

    public void Lights(){
        
    }

    
}
