package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.can.CANNotInitializedException;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
/**
 * This class runs the shooter, shooter lift, cam, and the optional limit switches
 * monitoring the orientation of the bottom frisbee in the shooter and the height
 * of the shooter
    Victor cam; * 
 * @author Andrew Vitkus
 */
public class Shooter {
    CANJaguar shooterWheel1;
    CANJaguar shooterWheel2;
    DigitalInput topLimit;
    DigitalInput bottomLimit;
    DigitalInput frisbeeSideCheck;
    DigitalInput camWatch;
    CANJaguar lift;
    Victor cam;

    Encoder encoder;
    
    boolean topSafe;
    boolean bottomSafe;
    boolean frisbeeUpright;
    boolean shooting;
    boolean autoAngle;
    
    double setValue;
    
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
        try {
            shooterWheel1 = new CANJaguar(shooterPort1);  // initalize the jag running the shooter
            shooterWheel1.configNeutralMode(CANJaguar.NeutralMode.kCoast);   // force the jag to coast
            shooterWheel1.changeControlMode(CANJaguar.ControlMode.kVoltage); // set the jag to run %Vbus control
            shooterWheel1.configEncoderCodesPerRev(250);
            shooterWheel1.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            shooterWheel1.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            shooterWheel1.setPID(.1, 0, 0);    // P=1, I=0.01, D=0.1
            shooterWheel1.enableControl(0);   // make the jag use the closed loop controller
            
            shooterWheel2 = new CANJaguar(shooterPort2);  // initalize the jag running the shooter
            shooterWheel2.configNeutralMode(CANJaguar.NeutralMode.kCoast);   // force the jag to coast
            shooterWheel2.changeControlMode(CANJaguar.ControlMode.kVoltage); // set the jag to run %Vbus control
            shooterWheel2.setPID(1, .01, .1);    // P=1, I=0.01, D=0.1
            shooterWheel2.enableControl();   // make the jag use the closed loop controller
            
            lift = new CANJaguar(liftPort);    // initalize the victor running the lift
            lift.configNeutralMode(CANJaguar.NeutralMode.kBrake);   // force the jag to coast
            lift.changeControlMode(CANJaguar.ControlMode.kPosition); // set the jag to run %Vbus control
            lift.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            lift.configEncoderCodesPerRev(500);
            lift.setSafetyEnabled(false);
            lift.setPID(500, 1, 10);    // P=1, I=0.01, D=0.1
            lift.enableControl(0);   // make the jag use the closed loop controller
            lift.setX(0);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
        
        
        cam = new Victor(camPort);  // initalize the victor running the cam
        
        stopLift();
        cam.set(0);
        
        setValue = 0;   // set the shooter wheel speed to 0
        
        topSafe = true; // assume the shooter is not at the max height
        bottomSafe = true;  // assume the shooter is not at the minimum height
        shooting = false;   // assume that the shooter is not shooting
        autoAngle = false;
    }
    
    /**
     * This creates a shooter with the shooter wheel, cam, lift, and frisbee orientation check.
     * 
     * @param shooterPort the CAN address of the Jaguar running the shooter wheel
     * @param camPort the PWM port the Victor running the cam is attached to
     * @param liftPort the PWM port the Victor running the lift is attached to
     * @param frisbeeSideCheckPort the DIO port the limit switch checking the frisbee orientation is attached to
     */
    public Shooter(int shooterPort1, int shooterPort2, int camPort, int liftPort, int frisbeeSideCheckPort) {
        this(shooterPort1, shooterPort2, camPort, liftPort);
        
        frisbeeSideCheck = new DigitalInput(frisbeeSideCheckPort);  // initalize the limit switch checking the frisbee orientation in the shooter
        
        startFrisbeeWatcher();  // start the thread to monitor the frisbee orientation
    }
    
    public Shooter(int shooterPort1, int shooterPort2, int camPort, int liftPort, int camCheckPort, Object whatever) {
        this(shooterPort1, shooterPort2, camPort, liftPort);
        
        camWatch = new DigitalInput(camCheckPort);  // initalize the limit switch checking the frisbee orientation in the shooter
    }
    
    /**
     * This creates a shooter with the shooter wheel, cam, and lift as well as
     * the top and bottom shooter height limits.
     * 
     * @param shooterPort the CAN address of the Jaguar running the shooter wheel
     * @param camPort the PWM port the Victor running the cam is attached to
     * @param liftPort the PWM port the Victor running the lift is attached to
     * @param topLimitPort the DIO port that the limit switch checking the maximum height of the shooter is attached to
     * @param bottomLimitPort the DIO port that the limit switch checking the minimum height of the shooter is attached to
     */
    public Shooter(int shooterPort1, int shooterPort2, int camPort, int liftPort, int topLimitPort, int bottomLimitPort) {
        this(shooterPort1, shooterPort2, camPort, liftPort);
        
        topLimit = new DigitalInput(topLimitPort);  // initalize the limit switch watching the max height of the shooter
        bottomLimit = new DigitalInput(bottomLimitPort);    // initalize the limit swtich watching the minimum height of the shooter
        
        startLimitWatcher();    // start the shooter lift height watcher
    }
    
    /**
     * This method starts the thread monitoring the orientation of the bottom frisbee in the shooter.
     */
    private void startFrisbeeWatcher() {
        /* 
         * montior the frisbee in a separate thread so we can keep the wheel sped up for
         * upright frisbees and slow for upside down ones regarless of the main thread's state
         * We can get away with the half second wait between checks because the cam spins this
         * long and only one shot can be made per cam motion
         */
        new Thread() {
            public void run() {
                while (true) {  // always watch the frisbees
                    if (shooting) {    // but only update when you aren't shooting
                        frisbeeUpright = frisbeeSideCheck.get();    // check if the frisbee is upright (switch not pressed) or upside down (switch pressed)
                    }
                    try {
                        Thread.sleep(500);  // wait for 0.5 seconds before checking again
                    } catch (InterruptedException ex) {
                        ex.printStackTrace();
                    }
                }
            }
        }.start();
    }
    
    /**
     * This method starts the thread watching the limit switches checking the maximum
     * and minimum height for the shooter.
     */
    private void startLimitWatcher() {
        /* 
         * I have the limit switches monitoring the top and bottom limits of the shooter's movement
         * being checked in their own thread because the main loop updates 50 time a second so the
         * methods to change the shooter's angle are calle this often as well. These methods check
         * if the limits are reached before moving and since they are only checked every 0.1 seconds
         * we can free cpu time that would be otherwise occupied.
         */
        new Thread() {
            public void run() {
                while (true) {  // always check the 
                    topSafe = topLimit.get();   // check if the shooter can be raised more
                    bottomSafe = bottomLimit.get(); // check if the shooter can lower more
                    try {
                        Thread.sleep(100);  // wait 0.1 seconds before checking again
                    } catch (InterruptedException ex) {
                        ex.printStackTrace();
                    }
                }
            }
        }.start();
    }
    
    /**
     * This method raises the lift. If the height limits are enabled, the top limit
     * must not be trigger or the lift will stop instead.
     */
    public void raise(double dist) {
        try {
            System.out.println("Raise lift");
            lift.setX(lift.getX() + dist);
        } catch (CANTimeoutException ex) {
        }
    }
    
    public void setLiftHeight(double height) {
        try {
            lift.setX(height);
        } catch (CANTimeoutException ex) {
        }
    }
    
    /**
     * This method lowers the lift. If the height limits are enabled, the bottom limit
     * must not be trigger or the lift will stop instead.
     */
    public void lower(double dist) {
        try {
            System.out.println("Lower lift");
            lift.setX(lift.getX() - dist);
        } catch (CANTimeoutException ex) {
        }
    }
    
    /**
     * This method stops moving the lift
     */
    public void stopLift() {
        try {
            if (lift.getControlMode() == CANJaguar.ControlMode.kPercentVbus) {
                if (!autoAngle && lift.getOutputVoltage() != 0) {  // is the lift already stopped?
                    System.out.println("Stopping lift");
                    lift.setX(0);    // if not, stop it
                }
            } else if (lift.getControlMode() == CANJaguar.ControlMode.kPosition) {
                lift.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
                stopLift();
                lift.changeControlMode(CANJaguar.ControlMode.kPosition);
            }
        } catch (CANTimeoutException ex) {
        }
    }
    
    /**
     * This method sets the speed at which to spin the shooter wheel. If this
     * is higher than the maximum possible speed or lower than the minimum, it
     * is changed to the closest value in range.
     * 
     * @param speed the speed to set the shooter to spin at
     */
    public void setShooterSpeed(double speed) {
        if (setValue != speed) {    // will the target speed even change?
            setValue = speed;   // if so, the target to the new value
            checkLimits();  // ensure the set value (speed) is wihin the limits
            try {
                shooterWheel1.setX(setValue, (byte)0x11);   // set the shooter wheel's first motor to go at the set speed
                shooterWheel2.setX(setValue, (byte)0x11);   // set the shooter wheel's second motor to go at the set speed
                System.out.println("Shooter speed: " + setValue);
                CANJaguar.updateSyncGroup((byte)0x11);
            } catch (CANNotInitializedException ex) {
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
        }
    }
    
    /**
     * This method increases the speed of the shooter by the given amount. The 
     * speed is limited as in the setShooterSpeed(speed) method.
     * 
     * @see #setShooterSpeed(double)
     * @param change the amount to raise the speed
     */
    public void raiseShooterSpeed(double change) {
        setShooterSpeed(setValue - change); // more negative speeds make the shooter spin faster forward
    }
    
    /**
     * This method decreases the speed of the shooter by the given amount. The 
     * speed is limited as in the setShooterSpeed(speed) method.
     * 
     * @see #setShooterSpeed(double)
     * @param change the amount to lower the speed
     */
    public void lowerShooterSpeed(double change) {
        setShooterSpeed(setValue + change); // more less negative speeds make the shooter spin slower forward
    }
    
    /**
     * This method stops the shooter wheel from spinning
     */
    public void stopShooter() {
        setShooterSpeed(0); // set the shooter wheel speed to 0
    }
    
    
    
    /**
     * This method attempts a shot.
     */
    public void shoot() {
        System.out.println("Shoot");
        shooting = true;    // record that a shot is being attempted
        Timer t = new Timer();
        while(shooterPercentError() > .1);
        cam.set(-1);    // rotate the cam to hit the frisbee
        t.start();
        while(t.get() < .1 || !camWatch.get()) { } // use a timer to wait for 0.5 seconds to pass
        
        cam.set(0); // stop the cam
        t.stop();
        shooting = false;   // record that the shot attempt is complete
    }
    
    private double shooterPercentError() {
        double set = 1;
        double current = 1;

        try {

            if (shooterWheel1.getControlMode() == CANJaguar.ControlMode.kSpeed) {
                set = setValue;
                current = shooterWheel1.getSpeed();
            }
        } catch (CANTimeoutException ex) {
        }

        return Math.abs((current - set) / set);
    }
    
    private double liftPercentError() {
        double set = 1;
        double current = 1;

        try {
            if (lift.getControlMode() == CANJaguar.ControlMode.kPosition) {
                set = setValue;
                current = shooterWheel1.getPosition();
            }
        } catch (CANTimeoutException ex) {
        }

        return Math.abs((current - set) / set);
    }
    
    public boolean shooterTargeted() {
        return shooterPercentError() < .1 && liftPercentError() < .1;
    }
    
    /**
     * This method ensures that the set value of the shooter wheel is within the
     * allowed range.
     */
    private void checkLimits() {
        
        try {
            if (shooterWheel1.getControlMode() == CANJaguar.ControlMode.kPercentVbus) {
                /*
                 * since the shooter spins forward with negative speeds, 0 is the largest
                 * possible speed without spinning backwards. this check makes sure the
                 * wheel can only be stopped or rotating forwards
                 */
                setValue = Math.min(setValue, 0);

                /*
                 * since -1 is the highest forward speed for the wheel, any more negative
                 * values cannot be used. this check makes sure -1 is the largest possible setting
                 */
                setValue = Math.max(setValue, -1);
            } else if (shooterWheel1.getControlMode() == CANJaguar.ControlMode.kVoltage) {
                setValue = Math.min(setValue, 0);
                setValue = Math.max(setValue, -1*shooterWheel1.getBusVoltage());
            }
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }
}
