package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeS;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;

public class SimShootNote {
  
  //Speaker translations
  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  private static final Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);
  private static final double shotSpeed = 5;
  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
  //Launcher position compared to the robot
  static Transform3d launcherTransform = new Transform3d( 0.292, 0, 0.1225, new Rotation3d(0, -Units.degreesToRadians(IntakeS.getDistance()-Constants.IntakeConstants.intakeOffset+8), 0.0));
  //Returns the robot pose
  public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    robotPoseSupplier = supplier;
  }
  static ArrayList<Pose3d> currentNotes = new ArrayList<Pose3d>();
  static boolean firstCycle = true;
  public static void resetNotes(){
    currentNotes.clear();
    for (int i = 0; i < Constants.DriveSimConstants.fieldNoteTranslations.length; i++){
      currentNotes.add(Constants.DriveSimConstants.fieldNoteTranslations[i]);
    }
  }
  
  public static void intake(int index){
      currentNotes.remove(index);
  }
  public static Pose3d[] getState(){
    Pose3d[] translator = new Pose3d[currentNotes.size()]; //TODO: currently dies if length less than 0
    for (int i = 0; i < currentNotes.size(); i++){
      translator[i] = currentNotes.get(i);
    }
    return translator;
  }
  public static void updateStates(){
    Pose3d[] translator = new Pose3d[currentNotes.size()]; //TODO: currently dies if length less than 0
    for (int i = 0; i < currentNotes.size(); i++){
      translator[i] = currentNotes.get(i);
    }
    Logger.recordOutput("NoteVisualizer", translator);
  }
  public static Command shoot() {
    
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  //Initial starting point
                  launcherTransform =  new Transform3d( 0.3, 0, 0.36, new Rotation3d(0, -Units.degreesToRadians(IntakeS.getDistance()-Constants.IntakeConstants.intakeOffset+8), 0.0));
                  final Pose3d startPose =
                      new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform);
                  final boolean isRed =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Red);
                  final Pose3d endPose =
                      new Pose3d(isRed ? redSpeaker : blueSpeaker, startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  firstCycle = true;
                  System.out.println("HERE");
                  return Commands.run(
                          () -> {
                            if (SimShootNote.firstCycle){
                              currentNotes.add(startPose.interpolate(endPose, timer.get() / duration));
                              SimShootNote.firstCycle = false;
                            }else{
                              currentNotes.remove(currentNotes.size()-1);
                              currentNotes.add(startPose.interpolate(endPose, timer.get() / duration));
                            }
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            currentNotes.remove(currentNotes.size()-1);
                          });
                 },
                Set.of())
            .ignoringDisable(true));
                }
}