package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.can.CANNotInitializedException;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.RobotDrive;

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
    protected Encoder rightEncoder;
    protected Encoder leftEncoder;
    private double deadBand;
    private boolean turbo = false;
    private final double distPerTick = 0.05;//Math.PI / 1080;
    private final double pointDistPerDeg = Math.PI / 30;
    private final double wideDistPerDeg = Math.PI / 15;
    private double leftAutoDriveTarget;
    private double rightAutoDriveTarget;

    /**
     * This creates a drive train controlled by 4 Jaguars on a CAN bus
     *
     * @param frontLeft the CAN address of the Jaguar running the front left
     * motor
     * @param rearLeft the CAN address of the Jaguar running the rear left motor
     * @param frontRight the CAN address of the Jaguar running the front right
     * motor
     * @param rearRight the CAN address of the Jaguar running the rear right
     * motor
     */
    public CANDrive(int frontLeft, int rearLeft, int frontRight, int rearRight, int ra, int rb, int la, int lb) {
        try {
            m_frontLeftMotor = new CANJaguar(frontLeft);    // initialize the jag running the front, left motor
            m_frontRightMotor = new CANJaguar(frontRight);  // initialize the jag running the front, right motor\

            m_frontLeftMotor.configNeutralMode(CANJaguar.NeutralMode.kBrake);   // force the jag to break
            m_frontRightMotor.configNeutralMode(CANJaguar.NeutralMode.kBrake);  // force the jag to break


            m_frontLeftMotor.configEncoderCodesPerRev(360);
            m_frontLeftMotor.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            m_frontLeftMotor.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            m_frontLeftMotor.setPID(1, .01, .1);

            m_frontRightMotor.configEncoderCodesPerRev(360);
            m_frontRightMotor.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            m_frontRightMotor.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            m_frontRightMotor.setPID(1, .01, .1);
            m_rearLeftMotor = new CANJaguar(rearLeft);  // initialize the jag running the rear, left motor
            m_rearRightMotor = new CANJaguar(rearRight);    // initialize the jag running the rear, right motor

            m_rearLeftMotor.configNeutralMode(CANJaguar.NeutralMode.kBrake);    // force the jag to break
            m_rearRightMotor.configNeutralMode(CANJaguar.NeutralMode.kBrake);   // force the jag to break
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }

        rightEncoder = new Encoder(ra, rb);
        leftEncoder = new Encoder(la, lb);
        rightEncoder.setDistancePerPulse(distPerTick);
        leftEncoder.setDistancePerPulse(distPerTick);
        rightEncoder.setReverseDirection(true);
        

        deadBand = 0;   // default to having no deadspace from the controller inputs
    }

    public void setAutoDriveMode() {
        try {
            m_frontLeftMotor.changeControlMode(CANJaguar.ControlMode.kVoltage);
            m_frontRightMotor.changeControlMode(CANJaguar.ControlMode.kVoltage);
            m_rearLeftMotor.changeControlMode(CANJaguar.ControlMode.kVoltage);
            m_rearRightMotor.changeControlMode(CANJaguar.ControlMode.kVoltage);

            /*m_frontLeftMotor.setPID(25, 0.01, 0.1);
             m_frontRightMotor.setPID(25, 0.01, 0.1);*/
            m_frontLeftMotor.setPID(1, 0.01, 0.1);
            m_frontRightMotor.setPID(1, 0.01, 0.1);
            m_rearLeftMotor.setPID(1, 0.01, 0.1);
            m_rearRightMotor.setPID(1, 0.01, 0.1);

            m_frontLeftMotor.enableControl(0);
            m_frontRightMotor.enableControl(0);
            m_rearLeftMotor.enableControl(0);
            m_rearRightMotor.enableControl(0);
        } catch (CANTimeoutException ex) {
        }
    }

    public void setTeleopMode() {
        try {
            m_frontLeftMotor.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            m_frontRightMotor.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            m_rearLeftMotor.changeControlMode(CANJaguar.ControlMode.kVoltage);
            m_rearRightMotor.changeControlMode(CANJaguar.ControlMode.kVoltage);

            m_frontLeftMotor.disableControl();
            m_frontRightMotor.disableControl();

            m_rearLeftMotor.setPID(1, .01, .1);
            m_rearLeftMotor.enableControl();

            m_rearRightMotor.setPID(1, .01, .1);
            m_rearRightMotor.enableControl();
            //m_rearLeftMotor.disableControl();
            //m_rearRightMotor.disableControl();
        } catch (CANTimeoutException ex) {
        }
    }

    /**
     * This method set the control mode for the drive Jaguars
     *
     * @param controlMode the new control mode
     */
    public void setControlMode(CANJaguar.ControlMode controlMode) {
        try {
            m_frontLeftMotor.changeControlMode(controlMode);    // set the control mode of the jag
            m_frontRightMotor.changeControlMode(controlMode);   // set the control mode of the jag
            if (m_rearLeftMotor != null) {
                m_rearLeftMotor.changeControlMode(controlMode); // set the control mode of the jag
                m_rearRightMotor.changeControlMode(controlMode);    // set the control mode of the jag
            }
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
     * This method drive in tank mode with each side of the drive train
     * controlled by a different axis on the joystick. The input values are
     * directly set from the axes giving a linear speed curve
     *
     * @param left the value for the left motors
     * @param right the value of the right motors
     */
    public void tankDrive(double left, double right) {
        tankDrive(left, right, false);
    }

    /**
     * This method drive in tank mode with each side of the drive train
     * controlled by a different axis on the joystick. The input values can be
     * squared to give a parabolic speed curve.
     *
     * @param left the value for the left motors
     * @param right the value of the right motors
     * @param scaleInputs whether to scale the inputs
     */
    public void tankDrive(double left, double right, boolean scaleInputs) {
        left = -limit(left);
        right = limit(right);

        try {
            if (scaleInputs && m_frontRightMotor.getControlMode() == CANJaguar.ControlMode.kPercentVbus) {
                left = scaleSpeed(left);
                right = scaleSpeed(right);
            }
        } catch (CANTimeoutException ex) {
        }

        if (right != 0 || left != 0) {
            String rightStr = Double.toString(right);
            String leftStr = Double.toString(left);
            System.out.println("Set - Right: " + rightStr.substring(0, Math.min(rightStr.length(), 5)) + ", Left: " + leftStr.substring(0, Math.min(leftStr.length(), 5)));
        }

        try {
            m_frontLeftMotor.setX(left, (byte) 0x08);
            m_frontRightMotor.setX(right, (byte) 0x08);
            m_rearLeftMotor.setX(m_frontLeftMotor.getOutputVoltage(), (byte) 0x08);
            m_rearRightMotor.setX(m_frontRightMotor.getOutputVoltage(), (byte) 0x08);
            CANJaguar.updateSyncGroup((byte) 0x08);
            //CANJaguar.updateSyncGroup((byte)0x09);
        } catch (CANNotInitializedException ex) {
        } catch (CANTimeoutException ex) {
        }
    }

    /**
     * This method restricts the set values for the drive train to the extremes
     * for each control mode.
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
        tankDrive(0, 0);
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
        dist = Math.abs(dist);
        speed = Math.abs(speed);
        
        if (rightEncoder.getDistance() < dist) {
            try {
                m_frontRightMotor.setX(-speed, (byte) 0x09);
                m_rearRightMotor.setX(-speed, (byte) 0x09);
            } catch (CANTimeoutException ex) {
            }
        } else {
            try {
                m_frontRightMotor.setX(0, (byte) 0x09);
                m_rearRightMotor.setX(0, (byte) 0x09);
            } catch (CANTimeoutException ex) {
            }
        }

        if (leftEncoder.getDistance() < dist) {
            try {
                m_frontLeftMotor.setX(speed, (byte) 0x09);
                m_rearLeftMotor.setX(speed, (byte) 0x09);
            } catch (CANTimeoutException ex) {
            }
        } else {
            try {
                m_frontLeftMotor.setX(0, (byte) 0x09);
                m_rearLeftMotor.setX(0, (byte) 0x09);
            } catch (CANTimeoutException ex) {
            }
        }

        try {
            CANJaguar.updateSyncGroup((byte) 0x09);
        } catch (CANTimeoutException ex) {
        }

        return rightEncoder.getDistance() < dist || leftEncoder.getDistance() < dist;
    }

    public boolean rightWideTurn(double angle, double speed) {
        double dist = angle * wideDistPerDeg;
        speed = Math.abs(speed);
        if (angle > 0) {
            if (rightEncoder.getDistance() < dist) {
                try {
                    m_frontRightMotor.setX(speed, (byte) 0x09);
                    m_rearRightMotor.setX(speed, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            } else {
                try {
                    m_frontRightMotor.setX(0, (byte) 0x09);
                    m_rearRightMotor.setX(0, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            }
        } else {
            speed = -speed;
            if (rightEncoder.getDistance() > dist) {
                try {
                    m_frontRightMotor.setX(speed, (byte) 0x09);
                    m_rearRightMotor.setX(speed, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            } else {
                try {
                    m_frontRightMotor.setX(0, (byte) 0x09);
                    m_rearRightMotor.setX(0, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            }
        }

        try {
            m_frontLeftMotor.setX(0, (byte) 0x09);
            m_rearLeftMotor.setX(0, (byte) 0x09);
        } catch (CANTimeoutException ex) {
        }

        try {
            CANJaguar.updateSyncGroup((byte) 0x09);
        } catch (CANTimeoutException ex) {
        }

        return rightEncoder.getDistance() < dist;
    }

    public boolean leftWideTurn(double angle, double speed) {
        double dist = angle * wideDistPerDeg;
        speed = Math.abs(speed);
        if (angle > 0) {
            if (leftEncoder.getDistance() < dist) {
                try {
                    m_frontLeftMotor.setX(speed, (byte) 0x09);
                    m_rearLeftMotor.setX(speed, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            } else {
                try {
                    m_frontLeftMotor.setX(0, (byte) 0x09);
                    m_rearLeftMotor.setX(0, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            }
        } else {
            speed = -speed;
            if (leftEncoder.getDistance() > dist) {
                try {
                    m_frontLeftMotor.setX(speed, (byte) 0x09);
                    m_rearLeftMotor.setX(speed, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            } else {
                try {
                    m_frontLeftMotor.setX(0, (byte) 0x09);
                    m_rearLeftMotor.setX(0, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            }
        }

        try {
            m_frontRightMotor.setX(0, (byte) 0x09);
            m_rearRightMotor.setX(0, (byte) 0x09);
        } catch (CANTimeoutException ex) {
        }

        try {
            CANJaguar.updateSyncGroup((byte) 0x09);
        } catch (CANTimeoutException ex) {
        }

        return leftEncoder.getDistance() < dist;
    }

    public boolean turnDegrees(double angle, double speed) {
        double dist = angle * pointDistPerDeg * angle > 0 ? 1 : -1;
        speed = Math.abs(speed);
        speed *= angle > 0 ? -1 : 1;
        if (angle > 0) {
            if (rightEncoder.getDistance() < dist) {
                try {
                    m_frontRightMotor.setX(speed, (byte) 0x09);
                    m_rearRightMotor.setX(speed, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            } else {
                try {
                    m_frontRightMotor.setX(0, (byte) 0x09);
                    m_rearRightMotor.setX(0, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            }
            speed = -speed;
            if (leftEncoder.getDistance() > dist) {
                try {
                    m_frontLeftMotor.setX(speed, (byte) 0x09);
                    m_rearLeftMotor.setX(speed, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            } else {
                try {
                    m_frontLeftMotor.setX(0, (byte) 0x09);
                    m_rearLeftMotor.setX(0, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            }
        } else {
            if (rightEncoder.getDistance() > dist) {
                try {
                    m_frontRightMotor.setX(speed, (byte) 0x09);
                    m_rearRightMotor.setX(speed, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            } else {
                try {
                    m_frontRightMotor.setX(0, (byte) 0x09);
                    m_rearRightMotor.setX(0, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            }
            speed = -speed;
            if (leftEncoder.getDistance() < dist) {
                try {
                    m_frontLeftMotor.setX(speed, (byte) 0x09);
                    m_rearLeftMotor.setX(speed, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            } else {
                try {
                    m_frontLeftMotor.setX(0, (byte) 0x09);
                    m_rearLeftMotor.setX(0, (byte) 0x09);
                } catch (CANTimeoutException ex) {
                }
            }
        }

        try {
            CANJaguar.updateSyncGroup((byte) 0x09);
        } catch (CANTimeoutException ex) {
        }

        return rightEncoder.getDistance() < dist || leftEncoder.getDistance() < dist;
    }

    private double scaleSpeed(double x) {
        boolean negative = x < 0;
        x = Math.abs(x);

        if (x >= .5) {
            x = (1.3 * x) - .3;
            if (!turbo) {
                x *= .8;
            }
        } else {
            x = 1.4 * (x * x);
        }

        

        if (negative) {
            x *= -1;
        }

        return x;
    }

    public void setTurbo(boolean enabled) {
        turbo = enabled;
    }

    public boolean getTurbo() {
        return turbo;
    }

    public boolean autoTargetMet() {
        return drivePercentError() < .1;
    }

    private double drivePercentError() {
        double set = 1;
        double current = 1;
        double error = 0;
        try {
            if (m_frontRightMotor.getControlMode() == CANJaguar.ControlMode.kPosition) {
                set = rightAutoDriveTarget;
                current = m_frontRightMotor.getPosition();
                error = Math.abs((current - set) / set);

                set = leftAutoDriveTarget;
                current = m_frontLeftMotor.getPosition();
                error += Math.abs((current - set) / set);
            }
        } catch (CANTimeoutException ex) {
        }

        return error;
    }
}
