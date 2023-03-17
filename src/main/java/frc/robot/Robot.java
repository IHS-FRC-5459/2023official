// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.dense.row.MatrixFeatures_CDRM;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.commands.ActiveLevel;
import frc.robot.subsystems.LEDSub;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive;
import frc.robot.commands.AutoRoutes.AutoBalance;
import frc.robot.commands.AutoRoutes.Middle;
import frc.robot.commands.AutoRoutes.Num1;
import frc.robot.commands.AutoRoutes.Num6;
import frc.robot.commands.Utilities.AutoIntakeCubes;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  static boolean overridePivot = false;
  static boolean overrideClaw = false;
  static boolean constantClawOverride = false;
  private Command m_autonomousCommand;
  
// sets up sendable chooser
  public static RobotContainer m_robotContainer;
  private Command driveCommand;

  private SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */



   
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //m_robotContainer = RobotContainer.getInstance();
    // sets u snedable chooser
    driveCommand = new Drive();
    autoChooser.addOption("Center", new Middle());
    autoChooser.addOption("Red Left", new Num6());
    autoChooser.addOption("Red Right", new Num1());
    autoChooser.addOption("Blue Left", new Num1());
    autoChooser.addOption("Blue Right", new Num6());
    autoChooser.addOption("balanceTEST", new AutoBalance());
    autoChooser.addOption("pickUpTEST", new AutoIntakeCubes());





    /* 
    autoChooser.addOption("Left Auto", new LeftAuto());
    autoChooser.addOption("Left Level", new LeftLevelAuto());
    autoChooser.addOption("Middle Auto", new MiddleAuto());
    autoChooser.addOption("Middle Level Auto", new MiddleLevelAuto());
    autoChooser.addOption("Place One", new PlaceOne());
    autoChooser.addOption("Place One Level", new PlaceOneLevel());
    autoChooser.addOption("Right Auto", new RightAuto());
    autoChooser.addOption("Right Level Auto", new RightLevelAuto());
    autoChooser.addOption("3 ft", new threeft());
    SmartDashboard.putData("Auto Mode", autoChooser);
*/
//SmartDashboard.putData("Auto Mode", autoChooser);
    //camera
    SmartDashboard.putData("Auto Mode", autoChooser);

    
    UsbCamera c = CameraServer.startAutomaticCapture();
   /*  MjpegServer server = new MjpegServer("cam server", 5459);
      server.setSource(c);
      server.setFPS(24);
      server.setResolution(640,480);
      */
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    double pitch = Robot.m_robotContainer.m_DriveSub.getPitch();
  SmartDashboard.putNumber("pitch", pitch);
    //SmartDashboard.putNumber("power", (Constants.drivetrain_kA * 9.81 * Math.sin(Math.toRadians(-pitch))));
    SmartDashboard.putNumber("enc distance", m_robotContainer.m_DriveSub.getAverageEncoderDistance());
    SmartDashboard.putNumber("left enc distance", m_robotContainer.m_DriveSub.getLeftEncoderDistance());
    SmartDashboard.putNumber("right enc distance", m_robotContainer.m_DriveSub.getRightEncoderDistance());
    SmartDashboard.putNumber("arm extend enc distance", m_robotContainer.m_ArmSub.getTicks());
    SmartDashboard.putNumber("arm pos", m_robotContainer.m_ArmSub.getPosition());
    SmartDashboard.putNumber("claw enc", m_robotContainer.m_ClawSub.getEncoder());
    SmartDashboard.putBoolean("Claw Clamp", constantClawOverride);
    m_robotContainer.m_ArmSub.moveToPosition(0.3, 10);


    //System.out.print("pitch " + Robot.m_robotContainer.m_DriveSub.getPitch());
   


  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_LEDSub.setBlinkIn(-0.45);
    m_autonomousCommand = autoChooser.getSelected();
 //m_autonomousCommand = new Middle();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_robotContainer.m_DriveSub.setDrive(0, 0);

    }
    driveCommand.schedule();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveCommand.execute();
    double xboxLeftTrigger = m_robotContainer.getxbox().getLeftTriggerAxis();
    double xboxRightTrigger = m_robotContainer.getxbox().getRightTriggerAxis();
    if(xboxLeftTrigger > 0.1)
    {
      constantClawOverride = false;
    }
    if(constantClawOverride){
      m_robotContainer.m_ClawSub.moveClaw(0, 2);

    } else {
      if(!overrideClaw) 
      {
        
        // if(xboxLeftTrigger >= 0.1){m_robotContainer.m_ClawSub.setClaw(-xboxLeftTrigger)}
        // else if(xboxRightTrigger >= 0.1){m_robotContainer.m_ClawSub.setClaw(xboxRightTrigger)}
        m_robotContainer.m_ClawSub.moveClaw(xboxLeftTrigger, xboxRightTrigger);
        //System.out.println(xboxLeftTrigger + ", " + xboxRightTrigger);
     }
    }



    if(!overridePivot){
      double y = m_robotContainer.getxbox().getLeftY();
      m_robotContainer.m_ArmSub.xboxPivot(y);
    }
   
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public static void setOverrideClaw(boolean v){
    overrideClaw = v;
  }

  public static void setOverridePivot(boolean v)
  {
    overridePivot = v;
  }

  public static void switchClawConstant(){
    constantClawOverride = !constantClawOverride;
  }

  public static void setClawConstant(boolean v)
  {
    constantClawOverride = v;
  }
  
}
