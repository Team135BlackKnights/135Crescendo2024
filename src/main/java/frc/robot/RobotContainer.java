// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.HangConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.HangC;
import frc.robot.commands.IntakeC;
import frc.robot.commands.OutakeC;
import frc.robot.commands.SetAngle;
import frc.robot.commands.SwerveC;
import frc.robot.commands.VariableAngle;
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
  private final LEDStripS ledStripS = new LEDStripS();
  private final SendableChooser<Command> autoChooser;

  public static XboxController driveController = new XboxController(0);
  public static XboxController manipController = new XboxController(1);

  JoystickButton aButton = new JoystickButton(driveController, 1);
  JoystickButton xButton = new JoystickButton(driveController, 3);
  JoystickButton yButton = new JoystickButton(manipController, 4);
  JoystickButton bButton = new JoystickButton(manipController, 2);
  POVButton povZero = new POVButton(driveController, 0);
  POVButton manipPOVZero = new POVButton(manipController, 0);
  POVButton manipPOV180 = new POVButton(manipController, 180);
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
    aButton.onTrue(swerveS.toggleAutoLockCommand());
    xButton.onTrue(new InstantCommand(() -> swerveS.zeroHeading()));
    yButton.onTrue(new VariableAngle(intakeS, outakeS, false));
    bButton.onTrue(new SetAngle(intakeS, outakeS, 13));
    povZero.onTrue(new HangMacroC(hangS, HangConstants.upperHookHeight));
    manipPOVZero.onTrue(new InstantCommand(() -> IntakeConstants.intakeOffset += 1));
    manipPOV180.onTrue(new InstantCommand(() -> IntakeConstants.intakeOffset -= 2));
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