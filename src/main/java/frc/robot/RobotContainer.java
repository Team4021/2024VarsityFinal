// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeForTime;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootForTime;
import frc.robot.commands.TrackingIntake;
import frc.robot.commands.Autos.Autos;
import frc.robot.subsystems.*;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ShooterSubsystem m_shoot = new ShooterSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final LimelightTags m_limelightTags = new LimelightTags();
  public final LimelightNotes m_limelightNotes = new LimelightNotes();
  public final IntermediateSubsystem m_inter = new IntermediateSubsystem();
  public static DigitalInput m_limitSwitch = new DigitalInput(0);
  public final Dashboard m_dashboard = new Dashboard(m_limitSwitch);

    private final SendableChooser<Command> autoChooser;

  // Joysticks
  private final CommandJoystick m_strafeController =
      new CommandJoystick(OIConstants.kStrafeControllerPort);
  private final CommandJoystick m_turnController =
      new CommandJoystick(OIConstants.kTurnControllerPort);
  private final GenericHID m_strafeGenericHID = 
      new GenericHID(OIConstants.kStrafeControllerPort);
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
      new JoystickButton(m_strafeGenericHID, 4);

  public void configMotors(){
    m_intake.configIntakeMotor();
    m_shoot.configShootMotor();
    m_inter.configIntermediateMotors();
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      // Configure the trigger bindings
      configureBindings();

      NamedCommands.registerCommand("Shoot", new ShootForTime(m_shoot, m_intake, m_inter, 0.5).withTimeout(0.5));
      NamedCommands.registerCommand("Intake", new IntakeForTime(m_intake, m_inter, 1, m_limitSwitch).withTimeout(1));
    //   NamedCommands.registerCommand("null", null);

      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);

      // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_strafeController.getY(), OIConstants.kJoystickDeadband),
              MathUtil.applyDeadband(m_strafeController.getX(), OIConstants.kJoystickDeadband),
              -MathUtil.applyDeadband(-m_turnController.getX(), OIConstants.kJoystickDeadband),
            //   -MathUtil.applyDeadband(-m_turnController.getY(), OIConstants.kJoystickDeadband),
              true,
              true),
          m_robotDrive));
    m_intake.setDefaultCommand(
        new RunCommand(
            () -> m_intake.noRunIntake(),
             m_intake));
    m_limelightNotes.setDefaultCommand(
        new RunCommand(
            () -> m_limelightNotes.defaultCommand(),
            m_limelightNotes));
    m_shoot.setDefaultCommand(
        new RunCommand(
            () -> m_shoot.noRunshoot(), m_shoot)
    );
    m_inter.setDefaultCommand(
        new RunCommand(
            () -> m_inter.noRunIntermediate(), m_inter
        )
    );
    m_dashboard.setDefaultCommand(
        new RunCommand(
            () -> m_dashboard.pushToDashboard(), m_dashboard
        )
    );
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
    m_leftButton2.whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));
    m_leftTrigger.whileTrue(new Intake(m_inter, m_intake, m_limitSwitch));
    // m_rightButton3.whileTrue(new RunCommand(
    // () -> m_robotDrive.drive(
    //     MathUtil.applyDeadband(m_strafeController.getY(), OIConstants.kJoystickDeadband),
    //     m_limelightNotes.changeXspeed(MathUtil.applyDeadband(m_strafeController.getX(), OIConstants.kJoystickDeadband), m_limitSwitch),
    //     -MathUtil.applyDeadband(-m_turnController.getX(), OIConstants.kJoystickDeadband),
    //     false, true),
    // m_robotDrive));
    m_rightButton3.whileTrue(new TrackingIntake(m_robotDrive, m_limelightNotes, m_limitSwitch));
    // m_rightTrigger.whileTrue(new Shoot(m_shoot, m_inter, m_intake));
         //       .andThen(new RunCommand(
        // () -> m_shoot.intermediate(),
        // m_shoot))
    m_rightTrigger.whileTrue(new Shoot(m_shoot, m_inter, m_intake));
    m_leftTrigger.whileTrue(new RunCommand(
          () -> m_robotDrive.drive(
              MathUtil.applyDeadband(m_strafeController.getY(), OIConstants.kJoystickDeadband),
            //   -MathUtil.applyDeadband(m_strafeController.getX(), OIConstants.kJoystickDeadband),
              0,
              -MathUtil.applyDeadband(-m_turnController.getX(), OIConstants.kJoystickDeadband),
              false, true),
          m_robotDrive));
    m_leftButton6.whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

        
    m_leftButton5.whileTrue(new RunCommand(
        () -> m_robotDrive.resetAbsolute(), m_robotDrive));
    // m_leftButton7.whileTrue(new RunCommand(() -> m_shoot.angleShooter(), m_shoot));
    m_leftButton4.whileTrue(new RunCommand(
        () -> m_shoot.angleShooterClose(), m_shoot));
    m_leftButton5.whileTrue(new RunCommand(
        () -> m_shoot.angleShooterFar(), m_shoot));
    m_leftButton7.whileTrue(new RunCommand(
        () -> m_shoot.resetToAbsolutePosition(), m_shoot));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//   public Command getAutonomousCommand() {
//     // An example command will be run in autonomous

//     return Autos.turnAuto(m_robotDrive, m_intake, m_inter, m_shoot, m_limitSwitch);
//   }
    public Command getAutonomousCommand(String m_chooser) {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        // if(m_chooser == "autoBalance"){
        // return Autos.balanceAuto(m_robotDrive, m_armSub, m_intakeSub);
        // if(m_chooser == "Straight"){
        //     return Autos.straightAuto(m_robotDrive, m_intake, m_inter, m_shoot, m_limitSwitch);
        // } else if(m_chooser == "ThreeBlue"){
        //     return Autos.turnAutoBlue(m_robotDrive, m_intake, m_inter, m_shoot, m_limitSwitch);
        // } else if(m_chooser == "ThreeRed"){
        //     return Autos.turnAutoRed(m_robotDrive, m_intake, m_inter, m_shoot, m_limitSwitch);
        // } else if(m_chooser == "New Auto"){
        //     return new PathPlannerAuto("New Auto");
        // } else{
        //     return Autos.nothing(m_robotDrive);
        // }
        return autoChooser.getSelected();

    }
}
