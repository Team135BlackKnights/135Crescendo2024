package frc.robot;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileWriter;
import java.io.IOException;
import java.io.File;
public class TxtWriter {

    public static void writeFile(String fileNameInDirectory, String dataSent){
        try {
            File writeFile = new File(Filesystem.getDeployDirectory(), fileNameInDirectory);
            final FileWriter txtWrite = new FileWriter(writeFile, true);
            txtWrite.write(dataSent);
            txtWrite.close();
            System.out.println("run");
        } catch (IOException e) {
           System.out.println("Can't print because"+ e.toString());
        } catch (Exception e){
            System.out.println("No clue what happened here but here's an error code:"+ e.toString());
        }

        
    }
}
