package frc.robot;
import java.io.File;
import java.util.Scanner;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.IOError;
public class DataHandler{
    public static FileOutputStream outputStream;
    public static OutputStreamWriter outputStreamWriter;
    String[] loggingArray;
    public static int id = 0;
    public static String directoryName = "E://logs/";
    public static File newFileName;
    /**
     * Creates a file on a USB attached to the rio.
     * Used in conjunction with https://colab.research.google.com/drive/1pj7j6u2x-s2kY2aWgmiyc5_2wwpUgusc. ONLY CALL ONCE OR IT'LL BRICK!
     * @return
     * FileName: The file name of the created file, or an error if it could not be created
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
                //TODO: Get this renaming to work

                Scanner renameScanner = new Scanner(createdFile);
                if (renameScanner.hasNext()){
                    id = Integer.parseInt(renameScanner.nextLine());
                    renameScanner.close();
                    System.out.println(newFileName);
                }
                
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
     * Creates an array of strings, each string represents a table entry to be logged. 
     * Recommended to start by logging the table heading names first though
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
     * Closes writer at the end, might not be used
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