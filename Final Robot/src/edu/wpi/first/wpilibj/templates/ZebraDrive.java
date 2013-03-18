/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;
/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2012. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.can.CANNotInitializedException;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.parsing.IUtility;

/**
 * Utility class for handling Robot drive based on a definition of the motor configuration.
 * The robot drive class handles basic driving for a robot. Currently, 2 and 4 motor standard
 * drive trains are supported. In the future other drive types like swerve and meccanum might
 * be implemented. Motor channel numbers are passed supplied on creation of the class. Those are
 * used for either the drive function (intended for hand created drive code, such as autonomous)
 * or with the Tank/Arcade functions intended to be used for Operator Control driving.
 */
public class ZebraDrive implements MotorSafety, IUtility {

    protected MotorSafetyHelper m_safetyHelper;

    /**
     * The location of a motor on the robot for the purpose of driving
     */
    public static class MotorType {

        /**
         * The integer value representing this enumeration
         */
        public final int value;
        static final int kFrontLeft_val = 0;
        static final int kFrontRight_val = 1;
        static final int kRearLeft_val = 2;
        static final int kRearRight_val = 3;
        /**
         * motortype: front left
         */
        public static final edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType kFrontLeft = new edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType(kFrontLeft_val);
        /**
         * motortype: front right
         */
        public static final edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType kFrontRight = new edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType(kFrontRight_val);
        /**
         * motortype: rear left
         */
        public static final edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType kRearLeft = new edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType(kRearLeft_val);
        /**
         * motortype: rear right
         */
        public static final edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType kRearRight = new edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType(kRearRight_val);

        private MotorType(int value) {
            this.value = value;
        }
    }
    public static final double kDefaultExpirationTime = 0.1;
    public static final double kDefaultSensitivity = 0.5;
    public static final double kDefaultMaxOutput = 1.0;
    protected static final int kMaxNumberOfMotors = 4;
    private final double distPerTick = 0.05;//Math.PI / 1080;
    private final double pointDistPerDeg = Math.PI / 30;
    private final double wideDistPerDeg = Math.PI / 15;
    private double deadband;
    protected final int m_invertedMotors[] = new int[4];
    protected double m_sensitivity;
    protected double m_maxOutput;
    protected SpeedController m_frontLeftMotor;
    protected SpeedController m_frontRightMotor;
    protected SpeedController m_rearLeftMotor;
    protected SpeedController m_rearRightMotor;
    protected boolean m_allocatedSpeedControllers;
    protected boolean turboRight = false;
    protected boolean turboLeft = false;
    protected Encoder rightEncoder;
    protected Encoder leftEncoder;
    protected static boolean kTank_Reported = false;

    /** Constructor for ZebraDrive with 2 motors specified with channel numbers.
     * Set up parameters for a two wheel drive system where the
     * left and right motor pwm channels are specified in the call.
     * This call assumes Jaguars for controlling the motors.
     * @param leftMotorChannel The PWM channel number on the default digital module that drives the left motor.
     * @param rightMotorChannel The PWM channel number on the default digital module that drives the right motor.
     */
    public ZebraDrive(final int leftMotorChannel, final int rightMotorChannel) {
        m_sensitivity = kDefaultSensitivity;
        m_maxOutput = kDefaultMaxOutput;
        m_frontLeftMotor = null;
        m_rearLeftMotor = new Jaguar(leftMotorChannel);
        m_frontRightMotor = null;
        m_rearRightMotor = new Jaguar(rightMotorChannel);
        for (int i = 0; i < kMaxNumberOfMotors; i++) {
            m_invertedMotors[i] = 1;
        }
        m_allocatedSpeedControllers = true;
        setupMotorSafety();
        tankDrive(0, 0);
    }

    /**
     * Constructor for ZebraDrive with 4 motors specified with channel numbers.
     * Set up parameters for a four wheel drive system where all four motor
     * pwm channels are specified in the call.
     * This call assumes Jaguars for controlling the motors.
     * @param frontLeftMotor Front left motor channel number on the default digital module
     * @param rearLeftMotor Rear Left motor channel number on the default digital module
     * @param frontRightMotor Front right motor channel number on the default digital module
     * @param rearRightMotor Rear Right motor channel number on the default digital module
     */
    public ZebraDrive(final int frontLeftMotor, final int rearLeftMotor,
            final int frontRightMotor, final int rearRightMotor) {
        m_sensitivity = kDefaultSensitivity;
        m_maxOutput = kDefaultMaxOutput;
        m_rearLeftMotor = new Jaguar(rearLeftMotor);
        m_rearRightMotor = new Jaguar(rearRightMotor);
        m_frontLeftMotor = new Jaguar(frontLeftMotor);
        m_frontRightMotor = new Jaguar(frontRightMotor);
        for (int i = 0; i < kMaxNumberOfMotors; i++) {
            m_invertedMotors[i] = 1;
        }
        m_allocatedSpeedControllers = true;
        setupMotorSafety();
        tankDrive(0, 0);
    }

    /**
     * Constructor for ZebraDrive with 2 motors specified as SpeedController objects.
     * The SpeedController version of the constructor enables programs to use the ZebraDrive classes with
     * subclasses of the SpeedController objects, for example, versions with ramping or reshaping of
     * the curve to suit motor bias or dead-band elimination.
     * @param leftMotor The left SpeedController object used to drive the robot.
     * @param rightMotor the right SpeedController object used to drive the robot.
     */
    public ZebraDrive(SpeedController leftMotor, SpeedController rightMotor) {
        if (leftMotor == null || rightMotor == null) {
            m_rearLeftMotor = m_rearRightMotor = null;
            throw new NullPointerException("Null motor provided");
        }
        m_frontLeftMotor = null;
        m_rearLeftMotor = leftMotor;
        m_frontRightMotor = null;
        m_rearRightMotor = rightMotor;
        m_sensitivity = kDefaultSensitivity;
        m_maxOutput = kDefaultMaxOutput;
        for (int i = 0; i < kMaxNumberOfMotors; i++) {
            m_invertedMotors[i] = 1;
        }
        m_allocatedSpeedControllers = false;
		setupMotorSafety();
		tankDrive(0, 0);
    }

    /**
     * Constructor for ZebraDrive with 4 motors specified as SpeedController objects.
     * Speed controller input version of ZebraDrive (see previous comments).
     * @param rearLeftMotor The back left SpeedController object used to drive the robot.
     * @param frontLeftMotor The front left SpeedController object used to drive the robot
     * @param rearRightMotor The back right SpeedController object used to drive the robot.
     * @param frontRightMotor The front right SpeedController object used to drive the robot.
     */
    public ZebraDrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
            SpeedController frontRightMotor, SpeedController rearRightMotor) {
        if (frontLeftMotor == null || rearLeftMotor == null || frontRightMotor == null || rearRightMotor == null) {
            m_frontLeftMotor = m_rearLeftMotor = m_frontRightMotor = m_rearRightMotor = null;
            throw new NullPointerException("Null motor provided");
        }
        m_frontLeftMotor = frontLeftMotor;
        m_rearLeftMotor = rearLeftMotor;
        m_frontRightMotor = frontRightMotor;
        m_rearRightMotor = rearRightMotor;
        m_sensitivity = kDefaultSensitivity;
        m_maxOutput = kDefaultMaxOutput;
        for (int i = 0; i < kMaxNumberOfMotors; i++) {
            m_invertedMotors[i] = 1;
        }
        m_allocatedSpeedControllers = false;
		setupMotorSafety();
		tankDrive(0, 0);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * drive the robot using two joystick inputs. The Y-axis will be selected from
     * each Joystick object.
     * @param leftStick The joystick to control the left side of the robot.
     * @param rightStick The joystick to control the right side of the robot.
     */
    public void tankDrive(GenericHID leftStick, GenericHID rightStick) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getY(), rightStick.getY(), true);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * drive the robot using two joystick inputs. The Y-axis will be selected from
     * each Joystick object.
     * @param leftStick The joystick to control the left side of the robot.
     * @param rightStick The joystick to control the right side of the robot.
     * @param scaleInputs Setting this parameter to true decreases the sensitivity at lower speeds
     */
    public void tankDrive(GenericHID leftStick, GenericHID rightStick, boolean scaleInputs) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getY(), rightStick.getY(), scaleInputs);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * This function lets you pick the axis to be used on each Joystick object for the left
     * and right sides of the robot.
     * @param leftStick The Joystick object to use for the left side of the robot.
     * @param leftAxis The axis to select on the left side Joystick object.
     * @param rightStick The Joystick object to use for the right side of the robot.
     * @param rightAxis The axis to select on the right side Joystick object.
     */
    public void tankDrive(GenericHID leftStick, final int leftAxis,
            GenericHID rightStick, final int rightAxis) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), true);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * This function lets you pick the axis to be used on each Joystick object for the left
     * and right sides of the robot.
     * @param leftStick The Joystick object to use for the left side of the robot.
     * @param leftAxis The axis to select on the left side Joystick object.
     * @param rightStick The Joystick object to use for the right side of the robot.
     * @param rightAxis The axis to select on the right side Joystick object.
     * @param scaleInputs Setting this parameter to true decreases the sensitivity at lower speeds
     */
    public void tankDrive(GenericHID leftStick, final int leftAxis,
            GenericHID rightStick, final int rightAxis, boolean scaleInputs) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), scaleInputs);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * This function lets you directly provide joystick values from any source.
     * @param leftValue The value of the left stick.
     * @param rightValue The value of the right stick.
     * @param scaleInputs Setting this parameter to true decreases the sensitivity at lower speeds
     */
    public void tankDrive(double leftValue, double rightValue, boolean scaleInputs) {
        
        if(!kTank_Reported){
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_Tank);
            kTank_Reported = true;
        }

        // square the inputs (while preserving the sign) to increase fine control while permitting full power
        leftValue = limit(leftValue);
        rightValue = limit(rightValue);
        
        leftValue = checkDeadband(leftValue);
        rightValue = checkDeadband(rightValue);
        
        if(scaleInputs) {
            leftValue = scaleValue(leftValue);
            rightValue = scaleValue(rightValue);
        }
        
        if (turboRight) {
            if (rightValue > 0) {
                rightValue = 1;
            } else if (rightValue < 0) {
                rightValue = -1;
            }
        }
        if (turboLeft) {
            if (leftValue > 0) {
                leftValue = 1;
            } else if (leftValue < 0) {
                leftValue = -1;
            }
        }
        //System.out.println("Right: " + rightValue + ", Left: " + leftValue);
        setLeftRightMotorOutputs(leftValue, rightValue);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * This function lets you directly provide joystick values from any source.
     * @param leftValue The value of the left stick.
     * @param rightValue The value of the right stick.
     */
    public void tankDrive(double leftValue, double rightValue) {
        tankDrive(leftValue, rightValue, true);
    }
    
    public void resetEncoders() {
        rightEncoder.reset();
        leftEncoder.reset();
    }

    public void startEncoders() {
        rightEncoder.start();
        leftEncoder.start();
    }

    public void stopEncoders() {
        rightEncoder.stop();
        leftEncoder.stop();
    }

    public boolean driveFeet(double dist, double speed) {
        limit(speed);
        dist = Math.abs(dist);
        speed = Math.abs(speed);
        
        if (rightEncoder.getDistance() < dist) {
                m_frontRightMotor.set(-speed);
                m_rearRightMotor.set(-speed);
        } else {
                m_frontRightMotor.set(0);
                m_rearRightMotor.set(0);
        }

        if (leftEncoder.getDistance() < dist) {
            m_frontLeftMotor.set(speed);
                m_rearLeftMotor.set(speed);
        } else {
             m_frontLeftMotor.set(0);
                m_rearLeftMotor.set(0);
        }

        return rightEncoder.getDistance() < dist || leftEncoder.getDistance() < dist;
    }

    public boolean rightWideTurn(double angle, double speed) {
        double dist = angle * wideDistPerDeg;
        speed = Math.abs(speed);
        if (angle > 0) {
            if (rightEncoder.getDistance() < dist) {
                m_frontRightMotor.set(speed);
                    m_rearRightMotor.set(speed);
            } else {
                m_frontRightMotor.set(0);
                    m_rearRightMotor.set(0);
            }
        } else {
            speed = -speed;
            if (rightEncoder.getDistance() > dist) {
                m_frontRightMotor.set(speed);
                    m_rearRightMotor.set(speed);
            } else {
                m_frontRightMotor.set(0);
                    m_rearRightMotor.set(0);
            }
        }
            m_frontLeftMotor.set(0);
            m_rearLeftMotor.set(0);

        return rightEncoder.getDistance() < dist;
    }

    public boolean leftWideTurn(double angle, double speed) {
        double dist = angle * wideDistPerDeg;
        speed = Math.abs(speed);
        if (angle > 0) {
            if (leftEncoder.getDistance() < dist) {
               m_frontLeftMotor.set(speed);
                    m_rearLeftMotor.set(speed);
            } else {
                    m_frontLeftMotor.set(0);
                    m_rearLeftMotor.set(0);
            }
        } else {
            speed = -speed;
            if (leftEncoder.getDistance() > dist) {
                m_frontLeftMotor.set(speed);
                    m_rearLeftMotor.set(speed);
            } else {
                 m_frontLeftMotor.set(0);
                    m_rearLeftMotor.set(0);
            }
        }

            m_frontRightMotor.set(0);
            m_rearRightMotor.set(0);
            
        return leftEncoder.getDistance() < dist;
    }

    public boolean turnDegrees(double angle, double speed) {
        double dist = angle * pointDistPerDeg * angle > 0 ? 1 : -1;
        speed = Math.abs(speed);
        speed *= angle > 0 ? -1 : 1;
        if (angle > 0) {
            if (rightEncoder.getDistance() < dist) {
                m_frontRightMotor.set(speed);
                    m_rearRightMotor.set(speed);
            } else {
                m_frontRightMotor.set(0);
                    m_rearRightMotor.set(0);
            }
            speed = -speed;
            if (leftEncoder.getDistance() > dist) {
                m_frontLeftMotor.set(speed);
                    m_rearLeftMotor.set(speed);
            } else {
                m_frontLeftMotor.set(0);
                    m_rearLeftMotor.set(0);
            }
        } else {
            if (rightEncoder.getDistance() > dist) {
                m_frontRightMotor.set(speed);
                    m_rearRightMotor.set(speed);
            } else {
                    m_frontRightMotor.set(0);
                    m_rearRightMotor.set(0);
            }
            speed = -speed;
            if (leftEncoder.getDistance() < dist) {
                    m_frontLeftMotor.set(speed);
                    m_rearLeftMotor.set(speed);
            } else {
                    m_frontLeftMotor.set(0);
                    m_rearLeftMotor.set(0);
            }
        }

        return rightEncoder.getDistance() < dist || leftEncoder.getDistance() < dist;
    }

    /** Set the speed of the right and left motors.
     * This is used once an appropriate drive setup function is called such as
     * twoWheelDrive(). The motors are set to "leftSpeed" and "rightSpeed"
     * and includes flipping the direction of one side for opposing motors.
     * @param leftOutput The speed to send to the left side of the robot.
     * @param rightOutput The speed to send to the right side of the robot.
     */
    public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
        if (m_rearLeftMotor == null || m_rearRightMotor == null) {
            throw new NullPointerException("Null motor provided");
        }

        byte syncGroup = (byte)0x80;

        if (m_frontLeftMotor != null) {
            m_frontLeftMotor.set(limit(leftOutput) * m_invertedMotors[edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType.kFrontLeft_val] * m_maxOutput, syncGroup);
        }
        m_rearLeftMotor.set(limit(leftOutput) * m_invertedMotors[edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType.kRearLeft_val] * m_maxOutput, syncGroup);

        if (m_frontRightMotor != null) {
            m_frontRightMotor.set(-limit(rightOutput) * m_invertedMotors[edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType.kFrontRight_val] * m_maxOutput, syncGroup);
        }
        m_rearRightMotor.set(-limit(rightOutput) * m_invertedMotors[edu.wpi.first.wpilibj.templates.ZebraDrive.MotorType.kRearRight_val] * m_maxOutput, syncGroup);

        if (m_safetyHelper != null) {
            m_safetyHelper.feed();
        }
    }

    /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    protected static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     */
    protected static void normalize(double wheelSpeeds[]) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        int i;
        for (i=1; i<kMaxNumberOfMotors; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) maxMagnitude = temp;
        }
        if (maxMagnitude > 1.0) {
            for (i=0; i<kMaxNumberOfMotors; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }
    
    private double scaleValue(double value) {
        boolean negative = value < 0;
        value = Math.abs(value);
        
        if (value < 0.5) {
            value = MathUtils.pow(value, 2) * 1.4;
        } else {
            value = (value * 1.3) - 0.3;
        }
        
        if (negative) {
            value *= -1;
        }
        
        return value;
    }
    
    private double checkDeadband(double value) {
        if (Math.abs(value) < deadband) {
            return 0;
        } else {
            return value;
        }
    }

    /**
     * Invert a motor direction.
     * This is used when a motor should run in the opposite direction as the drive
     * code would normally run it. Motors that are direct drive would be inverted, the
     * drive code assumes that the motors are geared with one reversal.
     * @param motor The motor index to invert.
     * @param isInverted True if the motor should be inverted when operated.
     */
    public void setInvertedMotor(edu.wpi.first.wpilibj.RobotDrive.MotorType motor, boolean isInverted) {
        m_invertedMotors[motor.value] = isInverted ? -1 : 1;
    }

    /**
     * Set the turning sensitivity.
     *
     * This only impacts the drive() entry-point.
     * @param sensitivity Effectively sets the turning sensitivity (or turn radius for a given value)
     */
    public void setSensitivity(double sensitivity)
    {
            m_sensitivity = sensitivity;
    }

    /**
     * Configure the scaling factor for using ZebraDrive with motor controllers in a mode other than PercentVbus.
     * @param maxOutput Multiplied with the output percentage computed by the drive functions.
     */
    public void setMaxOutput(double maxOutput)
    {
            m_maxOutput = maxOutput;
    }


    /**
     * Free the speed controllers if they were allocated locally
     */
    public void free() {
        if (m_allocatedSpeedControllers) {
            if (m_frontLeftMotor != null) {
                ((PWM) m_frontLeftMotor).free();
            }
            if (m_frontRightMotor != null) {
                ((PWM) m_frontRightMotor).free();
            }
            if (m_rearLeftMotor != null) {
                ((PWM) m_rearLeftMotor).free();
            }
            if (m_rearRightMotor != null) {
                ((PWM) m_rearRightMotor).free();
            }
        }
    }
    
    public void setRightTurbo(boolean enabled) {
        turboRight = enabled;
    }
    
    public boolean getRightTurbo() {
        return turboRight;
    }
    
    public void setLeftTurbo(boolean enabled) {
        turboLeft = enabled;
    }
    
    public boolean getLeftTurbo() {
        return turboLeft;
    }
    
    public void setDeadBand(double newBand) {
        deadband = newBand;
    }
    
    public double getDeadBand() {
        return deadband;
    }

    public void setExpiration(double timeout) {
        m_safetyHelper.setExpiration(timeout);
    }

    public double getExpiration() {
        return m_safetyHelper.getExpiration();
    }

    public boolean isAlive() {
        return m_safetyHelper.isAlive();
    }

    public boolean isSafetyEnabled() {
        return m_safetyHelper.isSafetyEnabled();
    }

    public void setSafetyEnabled(boolean enabled) {
        m_safetyHelper.setSafetyEnabled(enabled);
    }
    
    public String getDescription() {
        return "Robot Drive";
    }

    public void stopMotor() {
        if (m_frontLeftMotor != null) {
            m_frontLeftMotor.set(0.0);
        }
        if (m_frontRightMotor != null) {
            m_frontRightMotor.set(0.0);
        }
        if (m_rearLeftMotor != null) {
            m_rearLeftMotor.set(0.0);
        }
        if (m_rearRightMotor != null) {
            m_rearRightMotor.set(0.0);
        }
    }

    private void setupMotorSafety() {
        m_safetyHelper = new MotorSafetyHelper(this);
        m_safetyHelper.setExpiration(kDefaultExpirationTime);
        m_safetyHelper.setSafetyEnabled(true);
    }

    protected int getNumMotors()
    {
        int motors = 0;
        if (m_frontLeftMotor != null) motors++;
        if (m_frontRightMotor != null) motors++;
        if (m_rearLeftMotor != null) motors++;
        if (m_rearRightMotor != null) motors++;
        return motors;
    }
    
    public void initalizeRightEncoder(int ra, int rb) {
        rightEncoder = new Encoder(ra, rb);
        rightEncoder.setDistancePerPulse(distPerTick);
    }
    
    public void initalizeLeftEncoder(int la, int lb) {
        leftEncoder = new Encoder(la, lb);
        leftEncoder.setDistancePerPulse(distPerTick);
    }
}
