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
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStationLCD;

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
    Joystick rightStick;
    Joystick leftStick;
    CANDrive drive;
    DriverStationLCD dsLCD;
    Shooter shooter;
    Lift lift;
    PandaCom panda;
    double teleopIterCount;
    double autoIterCount;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        controller1 = new Joystick(1);  // first joystick setup as controller 1
        drive = new CANDrive(3, 6, 2, 1, 1, 2, 3, 4);  // setup drive train on CAN adresses 1, 2, 3, and 4
        lift = new Lift(3, 2, true);           // setup frisbee lift with front polycord motor on PWM 1 and rear on PWM 2, do use jaguars
        shooter = new Shooter(5, 4, 1, 7, 2, null);  // setup shooter with wheel on CAN address 5, lift on PWM 3

        dsLCD = DriverStationLCD.getInstance();

        drive.setDeadBand(.1);  // sets the deadband of the drive train to .1
        //drive.setSafetyEnabled(false);
        //drive.setExpiration(.5);

        // start the thread watching for data from the PandaBoard.

        panda = new PandaCom();
        //panda.monitor();
    }

    public void disabledInit() {
        //panda.pauseMonitor();    // pauses the PandaBoard monitor
    }

    /**
     * This function is run when the robot first enters the autonomous period
     */
    public void autonomousInit() {
        //panda.resumeMonitor();   // resume monitoring the PandBoard
        //drive.setControlMode(CANJaguar.ControlMode.kPosition);  // set the drive train to run based on position data
        autoIterCount = 0;  // zero the autonomous iteration counter
        drive.setAutoDriveMode();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        if (isAutonomous() && isEnabled()) {
            //run shooter at 9V
            shooter.setShooterSpeed(9);
        }
        if (isAutonomous() && isEnabled()) {
            //angle shooter
            shooter.setLiftHeight(3);
        }
        //shoot 3 frisbees
        if (isAutonomous() && isEnabled()) {
            shooter.shoot();
        }
        if (isAutonomous() && isEnabled()) {
            shooter.shoot();
        }
        if (isAutonomous() && isEnabled()) {
            shooter.shoot();
        }
        //lower shooter to 5V
        shooter.setShooterSpeed(5);
        //drive forward and turn on polycord
        lift.lift();
        drive.resetEncoders();
        drive.startEncoders();
        while (isAutonomous() && isEnabled() && drive.driveFeet(3, .75));
        //pick up frisbee or 2
        //2 may require driving forward, running only right side forward, stopping, then running right back
        drive.resetEncoders();
        while (isAutonomous() && isEnabled() && drive.rightWideTurn(90, .5));
        drive.resetEncoders();
        while (isAutonomous() && isEnabled() && drive.rightWideTurn(-90, .5));
        drive.stopEncoders();
        if (isAutonomous() && isEnabled()) {
            //raise shooter to 9V
            shooter.setShooterSpeed(9);
        }
        if (isAutonomous() && isEnabled()) {
            //angle shooter
            shooter.setLiftHeight(2.5);
        }
        //shoot a frisbee or 2
        if (isAutonomous() && isEnabled()) {
            shooter.shoot();
        }
        if (isAutonomous() && isEnabled()) {
            shooter.shoot();
        }
        if (isAutonomous() && isEnabled()) {
            //lower shooter to 5V
            shooter.setShooterSpeed(5);
        }
        //drive forward
        drive.resetEncoders();
        drive.startEncoders();
        while (isAutonomous() && isEnabled() && drive.driveFeet(3, .75));
        //pick up frisbee or 2
        //2 may require driving forward, running only right side forward, stopping, then running right back
        drive.resetEncoders();
        while (isAutonomous() && isEnabled() && drive.rightWideTurn(90, .5));
        drive.resetEncoders();
        while (isAutonomous() && isEnabled() && drive.rightWideTurn(-90, .5));
        drive.stopEncoders();

        if (isAutonomous() && isEnabled()) {
            //raise shooter to 9V
            shooter.setShooterSpeed(9);
        }

        if (isAutonomous() && isEnabled()) {
            //angle shooter
            shooter.setLiftHeight(2);
        }
        //shoot a frisbee or 
        if (isAutonomous() && isEnabled()) {
            shooter.shoot();
        }
        if (isAutonomous() && isEnabled()) {
            shooter.shoot();
        }
        //kill shooter
        shooter.stopShooter();
        //kill polycord
        lift.stop();
        //drive backwards
        drive.resetEncoders();
        drive.startEncoders();
        while (isAutonomous() && isEnabled() && drive.driveFeet(-10, 1));
        drive.stopEncoders();
    }

    /**
     * This function is called when the robot first enters operator control
     */
    public void teleopInit() {
        //panda.resumeMonitor();   // resume monitoring the PandaBoard
        drive.setTeleopMode();
        teleopIterCount = 0;    // zero the telop iteration counter
        //PandaCom.writeLine("This is text!");    // send some text to the PandaBoard to test things
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        /*try {
         System.out.println(shooter.lift.getX() + ": " + shooter.lift.getPosition());
         } catch (CANTimeoutException ex) {
         }*/
        
        if (!drive.getTurbo() && (controller1.getRawButton(11) || controller1.getRawButton(12))) {
            drive.setTurbo(true);
        } else {
            drive.setTurbo(false);
        }
        
        drive.tankDrive(controller1.getRawAxis(2) * -1, controller1.getRawAxis(4) * -1, true);    // drive with left Y-axis driving the left motors and right Y-axis driving the right
        //drive.tankDrive(leftStick.getY(), rightStick.getY());

        

        if (controller1.getRawAxis(6) == 1) {
            lift.lift();    // if the D-pad is up, run the lift up
        } else if (controller1.getRawAxis(6) == -1) {
            lift.reverse(); // if the D-pad is down, run the lift backwards
        } else if (controller1.getRawAxis(5) == 1) {
            lift.stop();    //if the D-pad is right, stop the lift
        }

        if (teleopIterCount % 5 == 0) {
            if (controller1.getRawButton(5)) { // angle the shooter up if the left bumper is pressed
                shooter.raise(1);
            } else if (controller1.getRawButton(6)) {   // angle the shooter down if the right bumper is pressed
                shooter.lower(1);
            }/* else {    // if neither bumper is pressed stop angling the shooter
             shooter.stopLift();
             }*/

            if (controller1.getRawButton(8)) { // if right trigger is pressed, increase the shooter speed by 5% V-bus
                shooter.raiseShooterSpeed(.1);
            } else if (controller1.getRawButton(7)) { // if left trigger is pressed, decrease the shooter speed by 5% V-bus
                shooter.lowerShooterSpeed(.1);
            } else if (controller1.getRawButton(2)) { // if the B button is pressed, stop the shooter
                shooter.stopShooter();
            }
        }

        if (controller1.getRawButton(3)) {
            shooter.shoot();    // if A button is pressed, shoot
        }

        if (controller1.getRawButton(1)) {
            shootFromInput();
        }

        String set = Double.toString(shooter.setValue);
        String cur = "0";
        try {
            cur = Double.toString(shooter.shooterWheel1.getSpeed());
        } catch (CANTimeoutException ex) {
        }
        System.out.println("Set: " + set.substring(0, Math.min(set.length(), 5)) + ", Cur: " + cur.substring(0, Math.min(cur.length(), 5)));


        teleopIterCount++; // increments the teleop iteration counter
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    }

    public void shootFromInput() {
        String line = panda.requestLine();

        int split = line.indexOf(',');
        String heightStr = line.substring(0, split);
        double heightValue = Double.parseDouble(heightStr);

        dsLCD.println(DriverStationLCD.Line.kUser1, 0, "dist: " + Double.toString(heightValue));

        split = heightStr.indexOf(',');
        String yAngleStr = heightStr.substring(1, split);
        double yAngleValue = Double.parseDouble(yAngleStr);

        dsLCD.println(DriverStationLCD.Line.kUser2, 0, "y angle: " + Double.toString(yAngleValue));

        split = yAngleStr.indexOf(',');
        String xAngleStr = yAngleStr.substring(1, split);
        double xAngleValue = Double.parseDouble(xAngleStr);

        dsLCD.println(DriverStationLCD.Line.kUser3, 0, "x angle: " + Double.toString(xAngleValue));

        drive.turnDegrees(Math.toDegrees(xAngleValue), .5);

        shooter.setLiftHeight(7);

        double speedValue = Double.parseDouble(line.substring(split));
        speedValue = Math.abs(speedValue) / 150;
        shooter.setShooterSpeed(-speedValue);

        while (!(shooter.shooterTargeted() && drive.autoTargetMet()));

        shooter.shoot();
    }
}
