package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStripS extends SubsystemBase{
    public InstantCommand setConstantColorCommand;
     int InitialLoopValue;
    AddressableLEDBuffer ledBuffer;
    AddressableLED leds;
    public LEDStripS(){
        leds = new AddressableLED(LEDConstants.ledPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        leds.setLength(ledBuffer.getLength());
        InitialLoopValue = 0;
    }
    public void setConstantColors(int h, int s, int v){//Essentially designed to make all the LEDs a constant color 
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setHSV(i, h, s, v);
         }
         
         leds.setData(ledBuffer);
    }
    public void setColorWave(int h, int s){//value is basically how dark it is, is controlled by the wave function
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            //sine wave used here because it oscillates periodically, in order to make sure that the value is acceptable we just truncate it
            final int value = InitialLoopValue+(int)Math.floor(Math.abs(Math.sin(i*(Math.PI/16)))*255); //Tweak this number to make the gradient more gentle or sharp (more is more gentle)
            // Set the value
            ledBuffer.setHSV(i, h, s, value);
          }
          // Increase by to make the rainbow "move"
          InitialLoopValue += (int)Math.floor(Math.abs(Math.sin(Math.PI/8))*255); //offset by one "notch" each time
          // Check bounds
          InitialLoopValue %= 255;
        }

    public InstantCommand allianceWave(){
        if (SwerveS.redIsAlliance){
        return new InstantCommand(() -> {setColorWave(LEDConstants.redH,LEDConstants.redS);}, this);   
        }
        else{
        return new InstantCommand(() -> {setColorWave(LEDConstants.blueH,LEDConstants.blueS);}, this);   
        }
        
    }
    public InstantCommand noteColor(){
        return new InstantCommand(() -> {setConstantColors(LEDConstants.noteH, LEDConstants.noteS, LEDConstants.noteV);},this);
    }
    }


