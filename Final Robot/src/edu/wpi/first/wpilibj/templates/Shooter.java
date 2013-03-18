package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.can.CANNotInitializedException;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 * This class runs the shooter, shooter lift, cam, and the optional limit
 * switches monitoring the orientation of the bottom frisbee in the shooter and
 * the height of the shooter Victor cam;
 *
 *
 * @author Andrew Vitkus
 */
public class Shooter {

    Jaguar shooterWheel1;
    Jaguar shooterWheel2;
    DigitalInput topLimit;
    DigitalInput bottomLimit;
    DigitalInput frisbeeSideCheck;
    DigitalInput camWatch;
    Victor lift;
    Victor cam;
    Encoder liftEncoder;
    Encoder shooterEncoder;
    boolean topSafe;
    boolean bottomSafe;
    boolean frisbeeUpright;
    boolean shooting;
    boolean autoAngle;
    boolean newSpeed;
    boolean newHeight;
    double setSpeed;
    double setHeight;
    double minValue = 0;
    double maxValue = -1;

    /**
     * This creates a shooter with only the shooter wheel, cam, and lift.
     *
     * @param shooterPort1
     * @param shooterPort2
     * @param camPort the PWM port the Victor running the cam is attached to
     * @param liftPort the PWM port the Victor running the lift is attached to
     */
    public Shooter(int shooterPort1, int shooterPort2, int camPort, int liftPort) {
        shooterWheel1 = new Jaguar(shooterPort1);  // initalize the jag running the shooter

        shooterWheel2 = new Jaguar(shooterPort2);  // initalize the jag running the shooter

        lift = new Victor(liftPort);

        cam = new Victor(camPort);  // initalize the victor running the cam

        stopLift();
        cam.set(0);

        setSpeed = 0;   // set the shooter wheel speed to 0

        topSafe = true; // assume the shooter is not at the max height
        bottomSafe = true;  // assume the shooter is not at the minimum height
        shooting = false;   // assume that the shooter is not shooting
        autoAngle = false;
    }

    public Shooter(int shooterPort1, int shooterPort2, int camPort, int liftPort, int camCheckPort, int liftA, int liftB, int liftI, int shooterA, int shooterB) {
        this(shooterPort1, shooterPort2, camPort, liftPort);

        liftEncoder = new Encoder(liftA, liftB);
        shooterEncoder = new Encoder(shooterA, shooterB);

        liftEncoder.setDistancePerPulse(12. / 15 / 500);
        shooterEncoder.setDistancePerPulse(1. / 250);

        liftEncoder.start();
        shooterEncoder.start();

        liftEncoder.setReverseDirection(true);

        setHeight = 0;
        setSpeed = 0;

        bottomLimit = new DigitalInput(liftI);

        camWatch = new DigitalInput(camCheckPort);  // initalize the limit switch checking the frisbee orientation in the shooter
    }

    /**
     * This method raises the lift. If the height limits are enabled, the top
     * limit must not be trigger or the lift will stop instead.
     */
    public void raise(double dist) {
        //System.out.println("Raise lift");
        setHeight += dist;
        checkLimits();
    }

    public void raise() {
        //System.out.println("Raise lift");
            lift.set(.8);
    }

    public void setLiftHeight() {
        double speed = 0;
        if (liftPercentError() > 0.1) {
            speed = 1;
            if (setHeight < liftEncoder.getDistance()) {
                speed *= -1;
            }
        } else if (liftPercentError() > 0.05) {
            speed = .8;
            if (setHeight < liftEncoder.getDistance()) {
                speed *= -1;
            }
        } else if (liftPercentError() > 0.01) {
            speed = .5;
            if (setHeight < liftEncoder.getDistance()) {
                speed *= -1;
            }
        }
        
        lift.set(speed);
    }

    public void setTargetLiftHeight(double height) {
        setHeight = height;
        checkLimits();
    }

    /**
     * This method lowers the lift. If the height limits are enabled, the bottom
     * limit must not be trigger or the lift will stop instead.
     */
    public void lower(double dist) {
        //System.out.println("Lower lift");
        setHeight -= dist;
        checkLimits();
    }
    
    public void lower() {
        if (bottomLimit.get()) {
            lift.set(-.8);
        } else {
            liftEncoder.reset();
            lift.set(0);
        }
    }

    /**
     * This method stops moving the lift
     */
    public void stopLift() {
        if (lift.get() != 0) {
            lift.set(0);
        }
    }

    /**
     * This method sets the speed at which to spin the shooter wheel. If this is
     * higher than the maximum possible speed or lower than the minimum, it is
     * changed to the closest value in range.
     *
     * @param speed the speed to set the shooter to spin at
     */
    public void setShooterSpeed() {
        //System.out.println("Shooter % error: " + Double.toString(shooterPercentError()));
        if (setSpeed == 0) {
            shooterWheel1.set(0);   // set the shooter wheel's first motor to go at the set speed
            shooterWheel2.set(0);   // set the shooter wheel's second motor to go at the set speed
            //System.out.println("Shooter speed: " + setSpeed);
        }
        if (shooterPercentError() > 0.01) {
            double vbus = shooterWheel1.get();
            if (shooterEncoder.getRate() < setSpeed) {
                if (vbus > -1) {
                    vbus -= 0.02;
                }
            } else if (shooterEncoder.getRate() > setSpeed) {
                if (vbus < 0) {
                    vbus += 0.01;
                }
            }

            shooterWheel1.set(vbus);   // set the shooter wheel's first motor to go at the set speed
            shooterWheel2.set(vbus);   // set the shooter wheel's second motor to go at the set speed
            //System.out.println("Shooter speed: " + setSpeed);
        }
    }

    public void setTargetShooterSpeed(double speed) {
        setSpeed = speed;
        checkLimits();
    }

    /**
     * This method increases the speed of the shooter by the given amount. The
     * speed is limited as in the setShooterSpeed(speed) method.
     *
     * @see #setShooterSpeed(double)
     * @param change the amount to raise the speed
     */
    public void raiseShooterSpeed(double change) {
        setSpeed -= change; // more negative speeds make the shooter spin faster forward
        checkLimits();
    }

    /**
     * This method decreases the speed of the shooter by the given amount. The
     * speed is limited as in the setShooterSpeed(speed) method.
     *
     * @see #setShooterSpeed(double)
     * @param change the amount to lower the speed
     */
    public void lowerShooterSpeed(double change) {
        setSpeed += change; // more less negative speeds make the shooter spin slower forward
        checkLimits();
    }

    /**
     * This method stops the shooter wheel from spinning
     */
    public void stopShooter() {
        setSpeed = 0; // set the shooter wheel speed to 0
        checkLimits();
    }

    /**
     * This method attempts a shot.
     */
    public void shoot() {
        //System.out.println("Shoot");
        new Thread() {
            public void run() {
                Timer t = new Timer();
                //while (shooterPercentError() > .1);
                cam.set(-1);    // rotate the cam to hit the frisbee
                t.start();
                while ((t.get() < .1 || !camWatch.get()) && t.get() < 1) {
                } // use a timer to wait for 0.5 seconds to pass

                cam.set(0); // stop the cam
                t.stop();
            }
        }.start();
    }

    private double shooterPercentError() {
        double set = 1;
        double current = 1;

        set = setSpeed;
        current = shooterEncoder.getRate();

        //System.out.println("Set speed: " + set + ", Current speed: " + current);
        if (set == 0 && shooterWheel1.get() == 0) {
            return 0;
        }
        return Math.abs((current - set) / set);
    }

    private double liftPercentError() {
        double set = setHeight;
        double current = liftEncoder.getDistance();
        if (set == 0 && liftEncoder.getDistance() == 0) {
            return 0;
        }
        return Math.abs((current - set) / set);
    }

    public boolean shooterTargeted() {
        return /*shooterPercentError() < .1 && */liftPercentError() < .01;
    }

    /**
     * This method ensures that the set value of the shooter wheel is within the
     * allowed range.
     */
    private void checkLimits() {
        setSpeed = Math.max(setSpeed, 0);
        setSpeed = Math.min(setSpeed, 200);
        setHeight = Math.max(setHeight, 3.5);
        setHeight = Math.min(setHeight, 13.5);
    }
}
