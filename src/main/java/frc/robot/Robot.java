// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Drive;
import frc.robot.commands.AutoRoutes.LeftAuto;
import frc.robot.commands.AutoRoutes.LeftLevelAuto;
import frc.robot.commands.AutoRoutes.MiddleAuto;
import frc.robot.commands.AutoRoutes.MiddleLevelAuto;
import frc.robot.commands.AutoRoutes.PlaceOne;
import frc.robot.commands.AutoRoutes.PlaceOneLevel;
import frc.robot.commands.AutoRoutes.RightAuto;
import frc.robot.commands.AutoRoutes.RightLevelAuto;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
// sets up sendable chooser
  public static RobotContainer m_robotContainer;
  private Command driveCommand;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

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
    autoChooser.addOption("Left Auto", new LeftAuto());
    autoChooser.addOption("Left Level", new LeftLevelAuto());
    autoChooser.addOption("Middle Auto", new MiddleAuto());
    autoChooser.addOption("Middle Level Auto", new MiddleLevelAuto());
    autoChooser.addOption("Place One", new PlaceOne());
    autoChooser.addOption("Place One Level", new PlaceOneLevel());
    autoChooser.addOption("Right Auto", new RightAuto());
    autoChooser.addOption("Right Level Auto", new RightLevelAuto());
    SmartDashboard.putData("Auto Mode", autoChooser);

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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();

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
      m_robotContainer.m_DriveSub.setDrive(0, 0);;

    }
    driveCommand.schedule();


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveCommand.execute();

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
}
