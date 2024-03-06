package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;

public class LEDStripS extends SubsystemBase{
    int initialLoopValue = 0;
    AddressableLEDBuffer ledBuffer;
    AddressableLED leds;

    public LEDStripS(){
        //creates LED objects
        leds = new AddressableLED(LEDConstants.ledPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        //sets length of the LED strip to buffer length
        leds.setLength(ledBuffer.getLength());
        
        //starts LED strips
        leds.start(); //FOR THE LOVE OF GOD PLEASE REMEMBER THIS IF YOU'RE GONNA CODE YOUR OWN SUBSYSTEM I SPENT LIKE 6 HOURS TROUBLESHOOTING AND IT DIDNT WORK BECAUSE OF THIS
        for (var i = 0; i < LEDConstants.sinePeriod; i++){
            LEDConstants.ledArray[i] = (int)Math.floor(Math.abs(Math.sin(((i*Math.PI/LEDConstants.sinePeriod)+initialLoopValue)))*255);
        }
        
        
    }

    @Override
    public void periodic() {
       /*NOTE: This code does not have a designated indicator for when the AutoLock program is running*/

        //if there is a note stored in the intake, set it to a constant note color
        if (IntakeS.noteIsLoaded()){
            
            setConstantColors(LEDConstants.greenH, LEDConstants.greenS, LEDConstants.greenV);
        }

        // if it's disabled make it so LEDs are off
        else if (DriverStation.isDisabled()) {
            //setColorWave(LEDConstants.goldH, LEDConstants.goldS, LEDConstants.disabledSinePeriod);
            setConstantColors(0, 0, 0);
        }

         else{

            //if it is trying to autolock
            if (SwerveS.autoLock){
                
                //if its locked on, set to constant green
                if (SwerveS.aprilTagVisible()){
                    setConstantColors(LEDConstants.greenH, LEDConstants.greenS, LEDConstants.greenV);
                }
                
                //if its not locked on, set to flashing green
                else{ 
                    setConstantColors(LEDConstants.greenH, LEDConstants.greenS, LEDConstants.greenV-70);
                }    
            } else {
                setConstantColors(0,0,0);
            }
        
            /* else{

                //If none of the previous conditions are met do the wave pattern with our alliance color
                if(SwerveS.redIsAlliance) {
                    setConstantColors(LEDConstants.redH, LEDConstants.redS, LEDConstants.redV);
                }
                    
                else {
                setConstantColors(LEDConstants.blueH, LEDConstants.blueS, LEDConstants.blueV);
                }
                } */
            }
        }
        

    public void setConstantColors(int h, int s, int v){//Essentially designed to make all the LEDs a constant color 
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setHSV(i, h, s, v);
        }
        leds.setData(ledBuffer);
    }
    public void setColorWave(int h, int s, double sinePeriod){//value is basically how dark it is, is controlled by the wave function
        Thread updateThread =   new Thread(
            () -> {
            for (var i = 0; i < ledBuffer.getLength(); i++) {
            /*The line of code below essentially just takes the number of the LED then multiplies it by pi divided by a variable that can be toggled to change wave size (sine is used because it oscillates). 
            To prevent a negative number from happening (as value only takes arguments in the range 0-255) and then rounds it down to ensure that the value outputted is an integer since this function only
            accepts integers (rounds down to prevent 256 from outputting). To get it to move, we use a loop value as a way to offset it.*/ 
            // Set the value
            ledBuffer.setHSV(i, h, s, LEDConstants.ledArray[i+initialLoopValue]);
            
        }
        
        

        // Increase the value computed in the sine function by pi/(the changable period) to make the gradient "move"
        initialLoopValue += 1; //offset by one "notch" each time
        
        
        //Check bounds
        initialLoopValue %= LEDConstants.sinePeriod;
    
        //sets data to buffer
        leds.setData(ledBuffer);
    });
    updateThread.setDaemon(true);
    updateThread.run();
    }

}
