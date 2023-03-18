// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Mechanism.Arm;
import frc.robot.commands.Utilities.AutoIntake;
import frc.robot.commands.Utilities.DriveStraight;
import frc.robot.commands.Utilities.MoveClaw;
import frc.robot.commands.Utilities.MoveExtend;
import frc.robot.commands.Utilities.MovePivot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.robot.commands.ActiveLevel;
import frc.robot.commands.Utilities.AutoScore;
import frc.robot.commands.Utilities.ConstantClaw;
import frc.robot.commands.Utilities.MoveIntake;
import frc.robot.commands.Utilities.RotateExtension;
import frc.robot.commands.Utilities.RotateLED;
import frc.robot.commands.Utilities.SlowSwitch;
import frc.robot.commands.Utilities.SwitchRobotDirection;
import frc.robot.commands.Utilities.ZeroExtendEncoder;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ClawSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.LEDSub;
import frc.robot.subsystems.RollerSub;
import edu.wpi.first.wpilibj.Joystick;
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
  public  ClawSub m_ClawSub = new ClawSub();
  public  ArmSub m_ArmSub = new ArmSub();
  public  DriveSub m_DriveSub = new DriveSub();
  public  RollerSub m_RollerSub = new RollerSub();
  public LEDSub m_LEDSub = new LEDSub();

  //joysticks

  public Joystick rightStick = null;
  public Joystick leftStick = null;
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
   * joysticks}.`````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````
   */
  private void configureBindings()  { 
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

   //  xboxOne.x().whileTrue(new MovePivot(0.2));
   //  xboxOne.y().whileTrue(new MovePivot(-0.2));

     xboxOne.x().debounce(0.1).onTrue(new AutoScore());
    xboxOne.y().debounce(0.1).whileTrue(new ConstantClaw());
    
     xboxOne.b().whileTrue(new MoveIntake(0.35));
     xboxOne.a().whileTrue(new MoveIntake(-0.35));
    xboxOne.leftBumper().debounce(0.1).whileTrue(new RotateExtension(-1));
    xboxOne.rightBumper().debounce(0.1).whileTrue(new RotateExtension(1));
  //xboxOne.leftBumper().whileTrue(new MoveExtend(-0.25));
    //xboxOne.rightBumper().whileTrue(new MoveExtend(0.3)); 
   xboxOne.start().whileTrue(new RotateLED());
   //xboxOne.back().whileTrue(new ZeroExtendEncoder());
   xboxOne.back().debounce(0.1).whileTrue(new RotateExtension("I"));

    JoystickButton balanceButton = new JoystickButton(rightStick, 12);
    balanceButton.whileTrue(new ActiveLevel(0.7, 0.6));
 
    JoystickButton directionButton = new JoystickButton(rightStick, 2);
    directionButton.debounce(0.05).toggleOnTrue(new SwitchRobotDirection());

     // directionButton.debounce(0.5).whileActiveOnce(((new SwitchDirection())));
    JoystickButton slowButton = new JoystickButton(rightStick, 1);
    slowButton.whileTrue(new ActiveLevel(1, 0.3));

    JoystickButton autobalanceButton = new JoystickButton(rightStick, 12);
    autobalanceButton.whileTrue(new ActiveLevel(0, 0));
     // xboxOne.x().whileTrue(new MoveExtend(0.25));
     // xboxOne.y().whileTrue(new MoveExtend(-0.25));
      //xboxOne.leftBumper().whileTrue(new MoveClaw(0.2));
      //xboxOne.rightBumper().whileTrue(new MoveClaw(-0.2));
    // xboxOne.leftBumper().whileTrue(new SetLED("yellow"));
   // xboxOne.leftBumper().whileTrue(new RotateLED()); 





    
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
