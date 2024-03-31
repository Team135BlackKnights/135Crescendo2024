package frc.robot;
import java.io.File;
import java.util.Scanner;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.IOError;

/**
 * Class used for logging data. Coded as an alternative to DataLogManager (so you don't get every update in NetworkTables logged, just the values you want), designed to be used for polynomial regression.
 * Saves data in columns instead of rows (each value for x is represented by the values below it)
 * Used in conjunction with https://colab.research.google.com/drive/1pj7j6u2x-s2kY2aWgmiyc5_2wwpUgusc to output a (somewhat) accurate polynomial model linking 2 variables together.
 * Can also be used as a way to save values from the RIO to a .txt file. 
 * To use this with the polynomial regression tool, delete all files in the "sample_data" folder, upload your files to the "sample_data" folder, and run the tool. 
 * You will need some knowledge in regression (r squared, residuals, etc) to interpret the result.
 */

 public class DataHandler{

    public static FileOutputStream outputStream;
    public static OutputStreamWriter outputStreamWriter;
    String[] loggingArray;
    public static int id = 0;
    public static String diskName= "/U";
    public static String directoryName = "";
    public static File newFileName;


    /**
     * Creates a file on a folder called "logs" (located on the USB attached to the rio) to log all values to. If there is no folder called "logs", it creates a folder. Only call one time (either in RobotInit or a subsystem constructor).
     * Using this function in simulation (instead of createLogFileInSimulation) will lead to an IOException of "the system cannot find the path specified". 
     * Use createLogFileInSimulation with a specific directory given to run this in simulation
     * If no USB drive is present, will just throw an error and not create a file. 
     */

    
    public static void createLogFileOnRIOUSB(){
        
        // "/U" is the default directory (drive name) for RoboRIO flash drives, create a new folder called "/U/Logs" if one doesn't exist
        //function based off of https://github.com/HoyaRobotics/InfiniteRecharge2020/blob/master/src/main/java/frc/robot/util/Logger.java

        //Creates the logs file in the specified drive if none is there
        directoryName = diskName + "/Logs";
        File directory = new File(directoryName);

        //This code isn't TECHNICALLY needed to make the directory (it auto checks if a folder is there), but i would like to make sure this is here just in case something happens
        if (!directory.exists()){
            boolean works = directory.mkdir();
            System.out.println(works);
        }

        try {  
            //Creates new file in the /U/Logs folder 
            String fileName = directoryName + "/Latest.txt";
            File createdFile = new File(fileName);
            
            //if a file named "latest" exists, rename "latest" to the id in its first line
            if (createdFile.exists()){
                Scanner renameScanner = new Scanner(createdFile);
                
                if (renameScanner.hasNext()){
                    id = Integer.parseInt(renameScanner.nextLine());
                    renameScanner.close();
                    System.out.println(newFileName);

                }
                
                //Create the new file object
                File newFileName = new File(directoryName + "/Log" + id + ".txt");
                createdFile.renameTo(newFileName);
            } 

            //Creates an actual file in the directory
            createdFile.createNewFile();
            
            //Adds 1 to the id then writes it to the first line, this is used to ensure no two logs have the same file number.
            id +=1; 
            
            //Makes the streamwriters for the written log file
            outputStream = new FileOutputStream(createdFile);
            outputStreamWriter = new OutputStreamWriter(outputStream);
            outputStreamWriter.write(id + "\r\n");
            outputStreamWriter.flush();
        } 
        
        //catch any errors
        catch (Exception e) {
            e.printStackTrace();
        }
        
        catch (IOError e){
            e.printStackTrace();
        }
    }


    /**
     * Writes values to file
     * Recommended to start by logging the table heading names first, polynomial regression tool handles this. If there is a time when you want to log data but want to ignore something, 
     * put null in as the value in the array (will output a string "null"). Regression calculator currently cannot handle this exception, as well as data relationships that have more than 2 variables (y = f(x) type functions)
     * Writes everything as a string, please convert values to strings before adding them to the array.
     *  @param tableHeadings the array of values to be logged, can be different from the values declared in the setUpLogOnUsb
     */
    public static void logData(String[] tableHeadings){
        //String that will be output to the writer
        String lineToBeSaved= "";
        
        //Adds each argument in the array to the string, adds a comma for separation (regression calculator uses this as well)
        for (String heading : tableHeadings){
            lineToBeSaved += (heading + ",");
        }

        //Removes last comma at the end
        lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length()-1));
        
        //Tries writing to the file, adds an error if it doesn't work
        try {
            outputStreamWriter.write(lineToBeSaved + "\r\n");
            outputStreamWriter.flush();
        } 
        
        //Catches errors
        catch (Exception e) {
            e.printStackTrace();
        }
        
        catch (IOError e){
            e.printStackTrace();
        }
    }


    /**
     * This function allows you to customize the directory that you send a log to in simulation. 
     * Designed to be used for simulations or other applications where the directory to be sent to is different from the default roboRIO USB Directory
     * All other functions in this class should work with the log and reader generated by this function.  
     * @param directory The folder where the simulated log will be written. Will need to be changed based on the device you're running simulation from and the directory where the logs are recorded.
     * @see DataHandler
     */
    public static void createLogFileinSimulation(String disk){
        diskName = disk;
        createLogFileOnRIOUSB();
    
    }
    
    
    /**
     * Flushes the writer (outputs last value) and then closes it. Does not need to be called. 
     */
    public static void closeWriter(){
        try {
            outputStreamWriter.flush();
            outputStreamWriter.close();
            
        }
        
        //Catches errors
        catch (Exception e) {
            e.printStackTrace();
        }
        
        catch (IOError e){
            e.printStackTrace();
        }
    }
}