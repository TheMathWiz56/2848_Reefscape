package frc.robot.subsystems;

import static frc.robot.Constants.LEDConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class Lights extends SubsystemBase{
    private final AddressableLED led = new AddressableLED(kPwmPort);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(kNumberOfLEDs);

    public Lights(){
        led.setLength(kNumberOfLEDs);
        led.start();
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
    
    public Command defaultRun(BooleanSupplier hasCoral, BooleanSupplier hasAlgae) {
        return run(() -> {
            if(hasCoral.getAsBoolean()) {
                LEDConstants.kWithCoral.applyTo(ledBuffer);
            } else if(hasAlgae.getAsBoolean()) {
                LEDConstants.kWithAlgae.applyTo(ledBuffer);
            } else {
                LEDConstants.kNormal.applyTo(ledBuffer);
            }
        });
    }

}