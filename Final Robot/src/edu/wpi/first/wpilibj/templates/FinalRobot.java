/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 * 
 * @author Andrew Vitkus
 */
public class FinalRobot extends IterativeRobot {
    Joystick controller1;
    
    CANDrive drive;
    
    Shooter shooter;
    
    Lift lift;
    
    double teleopIterCount;
    double autoIterCount;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        controller1 = new Joystick(1);  // first joystick setup as controller 1
        drive = new CANDrive(1,2,3,4);  // setup drive train on CAN adresses 1, 2, 3, and 4
        lift = new Lift(4, 3);           // setup frisbee lift with front polycord motor on PWM 1 and rear on PWM 2
        shooter = new Shooter(5, 1, 2);    // setup shooter with wheel on CAN address 5, lift on PWM 3
        
        drive.setDeadBand(.1);  // sets the deadband of the drive train to .1
        
        PandaCom.monitor(); // start the thread watching for data from the PandaBoard
    }
    
    public void disabledInit() {
        PandaCom.pauseMonitor();    // pauses the PandaBoard monitor
    }
    
    /**
     * This function is run when the robot first enters the autonomous period
     */
    public void autonomousInit() {
        PandaCom.resumeMonitor();   // resume monitoring the PandBoard
        drive.setControlMode(CANJaguar.ControlMode.kPosition);  // set the drive train to run based on position data
        autoIterCount = 0;  // zero the autonomous iteration counter
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }
    
    /**
     * This function is called when the robot first enters operator control
     */
    public void teleopInit() {
        PandaCom.resumeMonitor();   // resume monitoring the PandaBoard
        drive.setControlMode(CANJaguar.ControlMode.kPercentVbus );  // set the drive train to run based on %Vbus
        teleopIterCount = 0;    // zero the telop iteration counter
        PandaCom.writeLine("This is text!");    // send some text to the PandaBoard to test things
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        drive.tankDrive(controller1.getRawAxis(2) * -1, controller1.getRawAxis(4) * -1, true);    // drive with left Y-axis driving the left motors and right Y-axis driving the right

        if (controller1.getRawAxis(6) == 1) {
            lift.lift();    // if the D-pad is up, run the lift up
            
        } else if (controller1.getRawAxis(6) == -1) {
            lift.reverse(); // if the D-pad is down, run the lift backwards
        } else if (controller1.getRawAxis(5) == 1) {
            lift.stop();    //if the D-pad is right, stop the lift
        }
        
        if (teleopIterCount % 5 == 0) {
            if (controller1.getRawButton(5)) { // angle the shooter up if the left bumper is pressed
                shooter.raise();
            } else if (controller1.getRawButton(6)) {   // angle the shooter down if the right bumper is pressed
                shooter.lower();
            } else {    // if neither bumper is pressed stop angling the shooter
                shooter.stopLift();
            }
            
            if (controller1.getRawButton(8)) { // if right trigger is pressed, increase the shooter speed by 5% V-bus
                shooter.raiseShooterSpeed(.05); 
            } else if (controller1.getRawButton(7)) { // if left trigger is pressed, decrease the shooter speed by 5% V-bus
                shooter.lowerShooterSpeed(.05); 
            } else if (controller1.getRawButton(2)) { // if the B button is pressed, stop the shooter
                shooter.stopShooter();  
            }
        }
        
        if (controller1.getRawButton(3)) {
            shooter.shoot();    // if A button is pressed, shoot
        }
        
        teleopIterCount ++; // increments the teleop iteration counter
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
