package frc.robot;
import java.io.File;
import java.util.Scanner;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.IOError;

/**
 * Class used for logging data. Coded as an alternative to DataLogManager (so you don't get every NetworkTables update logged), designed to be used for polynomial regression. 
 * Used in conjunction with https://colab.research.google.com/drive/1pj7j6u2x-s2kY2aWgmiyc5_2wwpUgusc to output an accurate polynomial model linking 2 variables together.
 * Can also be used as a way to save values on a .txt file to the RIO. To use this with the polynomial regression tool, change the endings of all the files to .csv, and run it.
 * Saves data in columns instead of rows (each value for x is represented by the values below it). 
 */

 public class DataHandler{

    public static FileOutputStream outputStream;
    public static OutputStreamWriter outputStreamWriter;
    String[] loggingArray;
    public static int id = 0;
    public static String directoryName = "/U/logs/";
    public static File newFileName;
    /**
     * Creates a file on a USB attached to the rio to log all values to. ONLY CALL ONE TIME EVER (either in RobotInit or a subsystem constructor)
     */

    // "/U" is the default directory for RoboRIO flash drives, add new file to "/U/logs/"
    //function based off of https://github.com/HoyaRobotics/InfiniteRecharge2020/blob/master/src/main/java/frc/robot/util/Logger.java
    public static void setUpLogOnUSB(){

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
     * Recommended to start by logging the table heading names first, polynomial regression tool handles this. If there is a time when you want to log data but want to ignore something, put null in as the value in the array 
     * (will not be handled by regression calculator as of now)
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
     * Closes writer at the end, does not need to be used
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