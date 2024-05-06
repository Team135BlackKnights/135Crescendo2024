// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.HangConstants;
import frc.robot.commands.HangC;
import frc.robot.commands.IntakeC;
import frc.robot.commands.OutakeC;
import frc.robot.commands.SetAngle;
import frc.robot.commands.SwerveC;
import frc.robot.commands.VariableAngle;
import frc.robot.commands.VariableSpeed;
import frc.robot.commands.autoCommands.AutoLock;
import frc.robot.commands.autoCommands.AutonIntake;
import frc.robot.commands.autoCommands.FireShooter;
import frc.robot.commands.autoCommands.MoveIntake;
import frc.robot.subsystems.HangS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;
import frc.robot.subsystems.SwerveS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.HangMacroC;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.LEDStripS;
/**
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK OTHERWISE
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveS swerveS = new SwerveS();
  private final IntakeS intakeS = new IntakeS();
  private final OutakeS outakeS = new OutakeS();
  private final HangS hangS = new HangS();
  @SuppressWarnings("unused")
  private final LEDStripS ledStripS = new LEDStripS();
  private final SendableChooser<Command> autoChooser;

  public static XboxController driveController = new XboxController(0);
  public static XboxController manipController = new XboxController(1);

  JoystickButton aButton = new JoystickButton(driveController, 1);
  JoystickButton bButton = new JoystickButton(manipController, 2);
  JoystickButton xButton = new JoystickButton(driveController, 3);
  JoystickButton yButton = new JoystickButton(driveController, 4);
  static JoystickButton selectButton = new JoystickButton(driveController,7);
  static JoystickButton startButton = new JoystickButton(driveController,8);
  //POVButton povUpManip = new POVButton(driveController, 0);
  POVButton povRightManip = new POVButton(driveController, 90);
  POVButton povDownManip = new POVButton(driveController, 180);
  POVButton povLeftManip = new POVButton(driveController, 270);
  POVButton povUp = new POVButton(driveController, 0);
 // POVButton manipPOVZero = new POVButton(manipController, 0);
 // POVButton manipPOV180 = new POVButton(manipController, 180);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveS.setDefaultCommand(new SwerveC(swerveS));
    intakeS.setDefaultCommand(new IntakeC(intakeS));
    outakeS.setDefaultCommand(new OutakeC(outakeS));
    hangS.setDefaultCommand(new HangC(hangS));
    
    NamedCommands.registerCommand("DeployIntake", new MoveIntake(intakeS));
    NamedCommands.registerCommand("Lock Onto April Tags", new AutoLock(swerveS));
    NamedCommands.registerCommand("IntakeNote", new AutonIntake(intakeS));
    NamedCommands.registerCommand("ShootNote 3600 RPM", new FireShooter(outakeS, intakeS, 3600));
    NamedCommands.registerCommand("ShootNote 3000 RPM", new FireShooter(outakeS, intakeS, 3000));
    NamedCommands.registerCommand("Shoot From Anywhere", new VariableAngle(intakeS, outakeS, true));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    if (aButton.getAsBoolean() && isDriving()){
      swerveS.toggleAutoLockCommand();
    }
    if (yButton.getAsBoolean() && isDriving()){
      new InstantCommand(() -> swerveS.zeroHeading());
    }
    if (yButton.getAsBoolean() && isDriving()){
      new VariableSpeed(intakeS, outakeS, false);
    }
    if (bButton.getAsBoolean() && isDriving()){
      new SetAngle(intakeS, outakeS, 13);
    }
    
    //outake Tests
    startButton.and(aButton).whileTrue(outakeS.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    startButton.and(bButton).whileTrue(outakeS.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    startButton.and(xButton).whileTrue(outakeS.sysIdDynamic(SysIdRoutine.Direction.kForward));
    startButton.and(yButton).whileTrue(outakeS.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    //swerve DRIVE tests
    startButton.and(povUp).whileTrue(swerveS.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward));
    startButton.and(povRightManip).whileTrue(swerveS.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse));
    startButton.and(povDownManip).whileTrue(swerveS.sysIdDynamicDrive(SysIdRoutine.Direction.kForward));
    startButton.and(povLeftManip).whileTrue(swerveS.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse));
    
    //swerve TURNING tests

    selectButton.and(povUp).whileTrue(swerveS.sysIdQuasistaticTurn(SysIdRoutine.Direction.kForward));
    selectButton.and(povRightManip).whileTrue(swerveS.sysIdQuasistaticTurn(SysIdRoutine.Direction.kReverse));
    selectButton.and(povDownManip).whileTrue(swerveS.sysIdDynamicTurn(SysIdRoutine.Direction.kForward));
    selectButton.and(povLeftManip).whileTrue(swerveS.sysIdDynamicTurn(SysIdRoutine.Direction.kReverse));
   //manipController.y().and(manipController.start().negate()).onTrue(new VariableSpeed(intakeS, outakeS, false));
    //manipController.b().and(manipController.start().negate()).onTrue(new SetAngle(intakeS, outakeS, 13));
    if (povUp.getAsBoolean() && isDriving()){
      new HangMacroC(hangS, HangConstants.upperHookHeight);
      new SetAngle(intakeS, outakeS, 27);
    }
    //manipController.povUp().whileTrue(new SetAngle(intakeS, outakeS, 27));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
  public static boolean isDriving(){
    if (startButton.getAsBoolean() || selectButton.getAsBoolean()){
      return false; //currently doing a test
    }
    return true;
  }
}