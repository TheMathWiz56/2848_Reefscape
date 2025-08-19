package frc.robot.subsystems;

import static frc.robot.Constants.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.*;

public class Lights extends SubsystemBase{
    private final AddressableLED led = new AddressableLED(kPwmPort);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(kNumberOfLEDs);

    private LEDPattern currentPattern;

    public Lights(){
        led.setLength(kNumberOfLEDs);
        led.start();

        // Set the default pattern to fast scrolling Jesuit
        currentPattern = kFastScrollingJesuit;
    }

    @Override
    public void periodic(){
        led.setData(ledBuffer);
    }

    
    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(ledBuffer));
    }
    
}
