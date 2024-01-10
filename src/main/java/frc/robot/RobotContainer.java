// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Joysticks
  private final CommandJoystick m_strafeController =
      new CommandJoystick(OIConstants.kStrafeControllerPort);
  private final GenericHID m_strafeGenericHID = 
      new GenericHID(OIConstants.kStrafeControllerPort);
  private final CommandJoystick m_turnController =
      new CommandJoystick(OIConstants.kTurnControllerPort);
  private final GenericHID m_turnGenericHID = 
      new GenericHID(OIConstants.kTurnControllerPort);

private final JoystickButton m_rightTrigger =
      new JoystickButton(m_turnGenericHID, 1);
  private final JoystickButton m_leftTrigger =
      new JoystickButton(m_strafeGenericHID, 1);
  private final JoystickButton m_rightButton2 =
      new JoystickButton(m_turnGenericHID, 2);
  private final JoystickButton m_leftButton2 =
      new JoystickButton(m_strafeGenericHID, 2);
  private final JoystickButton m_rightButton3 =
      new JoystickButton(m_turnGenericHID, 3);
  private final JoystickButton m_leftButton3 =
      new JoystickButton(m_strafeGenericHID, 3);
  private final JoystickButton m_leftButton6 =
      new JoystickButton(m_strafeGenericHID, 6);
  private final JoystickButton m_leftButton7 = 
      new JoystickButton(m_strafeGenericHID, 7);
  private final JoystickButton m_leftButton5 = 
      new JoystickButton(m_strafeGenericHID, 5);
  private final JoystickButton m_rightButton4 =
      new JoystickButton(m_turnGenericHID, 4);
  private final JoystickButton m_rightButton5 =
      new JoystickButton(m_turnGenericHID, 5);
  private final JoystickButton m_leftButton4 =
      new JoystickButton(m_strafeGenericHID, 4);;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

      // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(-m_strafeController.getY(), OIConstants.kJoystickDeadband),
              -MathUtil.applyDeadband(-m_strafeController.getX(), OIConstants.kJoystickDeadband),
              -MathUtil.applyDeadband(-m_turnController.getX(), OIConstants.kJoystickDeadband),
              true, true),
          m_robotDrive));
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
    m_leftButton3.whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
