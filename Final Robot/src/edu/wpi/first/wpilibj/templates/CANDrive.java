package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 * This class runs the drive train for the robot 
 * 
 * @author Andrew Vitkus
 */
public class CANDrive {
    protected CANJaguar m_frontLeftMotor;
    protected CANJaguar m_rearLeftMotor;
    protected CANJaguar m_frontRightMotor;
    protected CANJaguar m_rearRightMotor;
    
    private double deadBand;
    
    private RobotDrive drive;
    
    /**
     * This creates a drive train controlled by 4 Jaguars on a CAN bus
     * 
     * @param frontLeft the CAN address of the Jaguar running the front left motor
     * @param rearLeft the CAN address of the Jaguar running the rear left motor
     * @param frontRight the CAN address of the Jaguar running the front right motor
     * @param rearRight the CAN address of the Jaguar running the rear right motor
     */
    public CANDrive(int frontLeft, int rearLeft, int frontRight, int rearRight) {
        try {
            m_frontLeftMotor = new CANJaguar(frontLeft);    // initialize the jag running the front, left motor
            m_rearLeftMotor = new CANJaguar(rearLeft);  // initialize the jag running the rear, left motor
            m_frontRightMotor = new CANJaguar(frontRight);  // initialize the jag running the front, right motor
            m_rearRightMotor = new CANJaguar(rearRight);    // initialize the jag running the rear, right motor
            
            m_frontLeftMotor.configNeutralMode(CANJaguar.NeutralMode.kBrake);   // force the jag to break
            m_rearLeftMotor.configNeutralMode(CANJaguar.NeutralMode.kBrake);    // force the jag to break
            m_frontRightMotor.configNeutralMode(CANJaguar.NeutralMode.kBrake);  // force the jag to break
            m_rearRightMotor.configNeutralMode(CANJaguar.NeutralMode.kBrake);   // force the jag to break

            drive = new RobotDrive(m_frontLeftMotor, m_rearLeftMotor, m_frontRightMotor, m_rearRightMotor); // setup a drivetrain with the CANJaguars
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
        
        deadBand = 0;   // default to having no deadspace from the controller inputs
    }
    
    /**
     * This method set the control mode for the drive Jaguars
     * 
     * @param controlMode the new control mode
     */
    public void setControlMode(CANJaguar.ControlMode controlMode) {
        try {
            m_frontLeftMotor.changeControlMode(controlMode);    // set the control mode of the jag
            m_rearLeftMotor.changeControlMode(controlMode); // set the control mode of the jag
            m_frontRightMotor.changeControlMode(controlMode);   // set the control mode of the jag
            m_rearRightMotor.changeControlMode(controlMode);    // set the control mode of the jag
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }
    
    /**
     * This method sets the range of values from the joystick that are treated
     * as zero by the drive train.
     * 
     * @param newDeadBand the new deadband for the drive train
     */
    public void setDeadBand(double newDeadBand) {
        deadBand = newDeadBand; // set the new controller deadband
    }
    
    /**
     * This method drive in tank mode with each side of the drive train controlled
     * by a different axis on the joystick. The input values are directly set
     * from the axes giving a linear speed curve
     * 
     * @param left the value for the left motors
     * @param right the value of the right motors
     */
    public void tankDrive(double left, double right) {
        tankDrive(left, right, false);
    }
    
    /**
     * This method drive in tank mode with each side of the drive train controlled
     * by a different axis on the joystick. The input values can be squared
     * to give a parabolic speed curve.
     * 
     * @param left the value for the left motors
     * @param right the value of the right motors
     * @param squaredInputs whether to square the inputs
     */
    public void tankDrive(double left, double right, boolean squaredInputs) {
        drive.tankDrive(limit(left), limit(right), squaredInputs);
    }
    
    /**
     * This method drive in arcade mode with the forward speed determined by one
     * axis on the joystick and turn by another. The input values are directly
     * set from the axes giving a linear speed curve
     * 
     * @param y the y value of the joystick
     * @param x the x value of the joystick
     */
    public void arcadeDrive(double y, double x) {
        arcadeDrive(y, x, false);
    }
    
    /**
     * This method drive in arcade mode with the forward speed determined by one
     * axis on the joystick and turn by another. The input values can be squared
     * to give a parabolic speed curve.
     * 
     * @param y the y value of the joystick
     * @param x the x value of the joystick
     * @param squaredInputs whether to square the inputs
     */
    public void arcadeDrive(double y, double x, boolean squaredInputs) {
        drive.arcadeDrive(limit(y), limit(x), squaredInputs);
    }
    
    /**
     * This method restricts the set values for the drive train to the extremes for
     * each control mode.
     * 
     * @param val the set drive value
     * @return the limited drive value
     */
    private double limit(double val) {
        try {
            CANJaguar.ControlMode ctrMode = m_frontLeftMotor.getControlMode(); // check the control mode
            if (ctrMode == CANJaguar.ControlMode.kPercentVbus) {    // if the control mode is %Vbus
                if (Math.abs(val) < deadBand) { // make sure that your setting is ouside the deadband
                    return 0;   // if it isn't, return 0 instead of the input
                } else { // if it is outside
                    val = Math.max(val, -1);    // make sure the setting is at least -1
                    val = Math.min(val, 1); // and make sure it is at most 1

                    return val; // then return the limited value
                }
            }
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
        
        return val; // otherwise, return the input
    }
    
    /**
     * This method stops the drive train.
     */
    public void stopMotor() {
        drive.stopMotor();  // call the equivalent method on the drivetrain
    }
    
    /**
     * This method sets the timeout of the drive train.
     * 
     * @param timeout the new timeout
     */
    public void setExpiration(double timeout) {
        drive.setExpiration(timeout);   // set the timout of the drivetrain
    }
    
    /**
     * This method gets the timeout of the drive train.
     * 
     * @return the drive train's timeout
     */
    public double getExpiration() {
        return drive.getExpiration();   // get the timout of drivetrain
    }
    
    /**
     * This method sets whether or not the safety of the drive train is enabled.
     * 
     * @param enabled the new setting for the safety
     */
    public void setSafetyEnabled(boolean enabled) {
        drive.setSafetyEnabled(enabled);    // set whether the drivetrain's safety is enabled
    }
    
    /**
     * This method checks if the drive train is alive.
     * 
     * @return is the drive train alive?
     */
    public boolean isAlive() {
        return drive.isAlive(); // is the drive train alive?
    }
}
