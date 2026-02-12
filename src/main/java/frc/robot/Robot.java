// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//these two imports are from the kitbot example
//may not be required for the actual implementation of the robot, but are included here for reference and potential use in the future
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();

        // NOTE: SHOULD PROBABLY BE REMOVED ONCE TESTING IS DONE, put good values in tuner constants (see commented out sections in TunerConstants)

        SmartDashboard.putNumber("Steer Gain P", 100);
        SmartDashboard.putNumber("Steer Gain I", 0);
        SmartDashboard.putNumber("Steer Gain D", 0.5);

        SmartDashboard.putNumber("Drive Gain P", 0.1);
        SmartDashboard.putNumber("Drive Gain I", 0);
        SmartDashboard.putNumber("Drive Gain D", 0);
    }

    
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
    @Override
    public void robotPeriodic() {
     // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    
        // TODO: REMOVE ONCE TUNING DONE!
        TunerConstants.setGains(
            SmartDashboard.getNumber("Steer Gain P", 100),
            SmartDashboard.getNumber("Steer Gain I", 0),
            SmartDashboard.getNumber("Steer Gain D", 0.5),

            SmartDashboard.getNumber("Drive Gain P", 0.1),
            SmartDashboard.getNumber("Drive Gain I", 0),
            SmartDashboard.getNumber("Drive Gain D", 0)
        );
    }
    
    /** This function is called once each time the robot enters Disabled mode. */
     @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
       if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

     /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

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
    public void simulationInit() {
    }

    @Override
    public void testExit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
