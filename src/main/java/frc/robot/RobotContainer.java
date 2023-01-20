// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ActiveLevel;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ClawSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.RollerSub;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
 // public static RobotContainer m_robotContainer = new RobotContainer();
  // The robot's subsystems and commands are defined here...
  //public final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final ClawSub m_ClawSub = new ClawSub();
  public final ArmSub m_ArmSub = new ArmSub();
  public final DriveSub m_DriveSub = new DriveSub();
  public final RollerSub m_RollerSub = new RollerSub();


  //joysticks
  
  private Joystick rightStick = null;
  private Joystick leftStick = null;
  private CommandXboxController xboxOne = new CommandXboxController(2);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
  //    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  /*private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      */

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    // redefining joysticks
    /*try{
      new CommandXboxController(2);
        }catch(Exception e){
      System.out.println("Error with Xbox controller init");
    }*/
    try
    {
      rightStick = new Joystick(1);
    }catch(Exception e){
      System.out.println("Error w/ right joystick init");
    }

    try
    {
      leftStick = new Joystick(0);
    }catch(Exception e){
      System.out.println("Error w/ left joystick init");
    }

    

    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings()  { 
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

      xboxOne.b().whileTrue(new ActiveLevel(0, 1.5));    

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }



  public Joystick getleftStick() {
    return leftStick;
}

public Joystick getrightStick() {
    return rightStick;
}

public CommandXboxController getxbox() {
  return xboxOne;
}
}
