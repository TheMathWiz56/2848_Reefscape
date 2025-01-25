package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    
    AddressableLED leds = new AddressableLED(1);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(150);

    public LEDs() {
        leds.setLength(150);

        leds.setData(ledBuffer);
        leds.start();

        LEDPattern red = LEDPattern.solid(Color.kRed);

        red.applyTo(ledBuffer);
        leds.setData(ledBuffer);
    }

    

}
