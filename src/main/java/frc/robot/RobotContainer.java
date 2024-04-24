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
import frc.robot.subsystems.CameraS;
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
  @SuppressWarnings("unused") //only periodic ledStrip is used, so don't care.
  private final LEDStripS ledStripS = new LEDStripS();
  @SuppressWarnings("unused") //only periodic camera updates is used, so don't care.
  private final CameraS cameraS = new CameraS();
  private final SendableChooser<Command> autoChooser;
  public static XboxController driveController = new XboxController(0);
  public static XboxController manipController = new XboxController(1);

  JoystickButton aButton = new JoystickButton(driveController, 1);
  JoystickButton xButton = new JoystickButton(driveController, 3);
  JoystickButton yButton = new JoystickButton(manipController, 4);
  JoystickButton bButton = new JoystickButton(manipController, 2);
  POVButton povZero = new POVButton(driveController, 0);
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
    NamedCommands.registerCommand("IntakeNote", new AutonIntake(intakeS,swerveS));
    NamedCommands.registerCommand("Shoot From Anywhere", new VariableAngle(intakeS, outakeS, true));
    NamedCommands.registerCommand("Hold Outake Ready", new FireShooter(outakeS));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    if (aButton.getAsBoolean() && !manipController.getStartButton()){
      swerveS.toggleAutoLockCommand();
    }
    if (yButton.getAsBoolean() && !manipController.getStartButton()){
      new InstantCommand(() -> swerveS.zeroHeading());
    }
    if (yButton.getAsBoolean() && !manipController.getStartButton()){
      new VariableSpeed(intakeS, outakeS, false);
    }
    if (bButton.getAsBoolean() && !manipController.getStartButton()){
      new SetAngle(intakeS, outakeS, 13);
    }
   //manipController.y().and(manipController.start().negate()).onTrue(new VariableSpeed(intakeS, outakeS, false));
    //manipController.b().and(manipController.start().negate()).onTrue(new SetAngle(intakeS, outakeS, 13));
    povZero.onTrue(new HangMacroC(hangS, HangConstants.upperHookHeight));
    povZero.onTrue(new SetAngle(intakeS, outakeS, 27));
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
}