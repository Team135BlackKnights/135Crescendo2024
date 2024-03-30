package frc.robot;
import java.io.File;
import java.util.Scanner;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.IOError;

/**
 * IMPORTANT: Requires a folder named logs on the destination drive/directory to prevent errors, or you might get an IOException where the system cannot find the path.
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
    public static String directoryName = "/U/logs/";
    public static File newFileName;
    /**
     * Creates a file on a USB attached to the rio to log all values to. Only call one time (either in RobotInit or a subsystem constructor).
     * Using this function in simulation (instead of createLogFileInSimulation) will lead to an IOException of "the system cannot find the path specified". 
     * Use createLogFileInSimulation with a specific directory given to run this in simulation
     */

    // "/U" is the default directory (drive name) for RoboRIO flash drives, add new file to "/U/logs/"
    //function based off of https://github.com/HoyaRobotics/InfiniteRecharge2020/blob/master/src/main/java/frc/robot/util/Logger.java
    public static void createLogFileOnRIOUSB(){

        try {
        
            //Creates new file 
            String fileName = directoryName + "Latest.txt";
            File createdFile = new File(fileName);
            
            //if a file named "latest" exists, rename "latest" to the id in its first line
            if (createdFile.exists()){

                Scanner renameScanner = new Scanner(createdFile);
                if (renameScanner.hasNext()){
                    id = Integer.parseInt(renameScanner.nextLine());
                    renameScanner.close();
                    System.out.println(newFileName);
                }

                //Create the new file
                File newFileName = new File(directoryName + "Log" + id + ".txt");
                createdFile.renameTo(newFileName);

            } 

            createdFile.createNewFile();
    
            id +=1; 
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
     * @param tableHeadings the array of values to be logged, can be different from the values declared in the setUpLogOnUsb
     */
    public static void logData(String[] tableHeadings){
        
        String lineToBeSaved= "";

        for (String heading : tableHeadings){
            lineToBeSaved += (heading + ",");
        }
        lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length()-1));
        try {
            outputStreamWriter.write(lineToBeSaved + "\r\n");
            outputStreamWriter.flush();
        } 
        catch (Exception e) {
            e.printStackTrace();
        }
        catch (IOError e){
            e.printStackTrace();
        }
    }
    /**
     * IMPORTANT: You need a folder called "logs" made in the directory you send the simulated file or you might get an IOException where the system cannot find the path.
     * This function allows you to customize the directory that you send a log to in simulation. 
     * Designed to be used for simulations or other applications where the directory to be sent to is different from the default roboRIO USB Directory
     * All other functions in this class should work with the log and reader generated by this function.  
     * @param directory The folder where the simulated log will be written. Will need to be changed based on the device you're running simulation from and the directory where the logs are recorded.
     * @see DataHandler
     */
    public static void createLogFileinSimulation(String directory){
        directoryName = directory;
        createLogFileOnRIOUSB();
    }
    /**
     * Closes writer at the end, most likely will not be used
     */
    public static void closeWriter(){
        try {
            outputStreamWriter.close();
        } 
        catch (Exception e) {
            e.printStackTrace();
        }
        catch (IOError e){
            e.printStackTrace();
        }
    }
}