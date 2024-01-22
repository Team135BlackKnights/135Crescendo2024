// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeC;
import frc.robot.commands.OutakeC;
//import frc.robot.commands.Autos;
import frc.robot.commands.SwerveC;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;
import frc.robot.subsystems.SwerveS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.LEDStripS;
/**
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK OTHERWISE
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveS swerveS = new SwerveS();
  private final IntakeS intakeS = new IntakeS();
  private final OutakeS outakeS = new OutakeS();
  private final LEDStripS ledStripS = new LEDStripS();
  private final DigitalInput intakeLimitSwitch = new DigitalInput(IntakeConstants.intakeLimitSwitchID); //the intake limit switch
  private final SendableChooser<Command> autoChooser;

  public static XboxController driveController = new XboxController(0);
  Trigger ringInIntake = new Trigger(intakeLimitSwitch::get); //binds trigger to the output of the limitSwitch
  JoystickButton aButton = new JoystickButton(driveController, 1);
  JoystickButton bButton = new JoystickButton(driveController, 2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveS.setDefaultCommand(new SwerveC(swerveS));
    intakeS.setDefaultCommand(new IntakeC(intakeS));
    outakeS.setDefaultCommand(new OutakeC(outakeS));
    //NamedCommands.registerCommand("SpinMotor", new SpinMotor(randomMotor, 2));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",autoChooser);
    // Configure the trigger bindings
    configureBindings();
    //ringInIntake.onTrue(ledStripS.setConstantColors(0, 0, 0));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    aButton.onTrue(swerveS.toggleAutoLockCommand());
    bButton.onTrue(intakeS.toggleIntakeDirectionCommand());
    ringInIntake.onTrue(ledStripS.noteColor());
    ringInIntake.onFalse(ledStripS.allianceWave());
    
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
