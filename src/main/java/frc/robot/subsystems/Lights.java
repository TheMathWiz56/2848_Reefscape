package frc.robot.subsystems;

import static frc.robot.Constants.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.*;

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

    /** Sets the lights to blink orange, indicating that a command is being run.
     * @return Command
     */
    public Command inAction(){
        return startEnd(() -> kOrange.applyTo(ledBuffer), () -> kFastScrollingJesuit.applyTo(ledBuffer));
    }

    /** Sets the lights to blink orange, indicating that a command has finished.
     * @return Command
     */
    public Command actionComplete(){
        return startEnd(() -> kGreenBlink.applyTo(ledBuffer), () -> kFastScrollingJesuit.applyTo(ledBuffer))
            .withTimeout(Seconds.of(1));
    }

    public Command holdState(){
        return Commands.idle(this);
    }
    
}
