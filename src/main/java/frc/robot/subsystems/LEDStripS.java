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
            /*The line of code below essentially just takes the number of the LED then multiplies it by pi divided by a variable that can be toggled to change wave size (sine is used because it oscillates). 
            To prevent a negative number from happening (as value only takes arguments in the range 0-255) and then rounds it down to ensure that the value outputted is an integer since this function only
            accepts integers (rounds down to prevent 256 from outputting). To get it to move, we use a loop value as a way to offset it.*/ 
            final int value = (int)Math.floor(Math.abs(Math.sin((InitialLoopValue+i)*(Math.PI/LEDConstants.sinePeriod)))*255); //Tweak this number to make the gradient more gentle or sharp (more is more gentle)
            // Set the value
            ledBuffer.setHSV(i, h, s, value);
          }

          // Increase the value computed in the sine function by pi/16 to make the gradient "move"
          InitialLoopValue += 1/LEDConstants.sinePeriod; //offset by one "notch" each time
          // Check bounds
          if (InitialLoopValue >= 1){
            InitialLoopValue = 0;
          }
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


