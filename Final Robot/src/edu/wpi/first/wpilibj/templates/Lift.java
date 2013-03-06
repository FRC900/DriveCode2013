package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

/**
 *
 * @author Andrew Vitkus
 */
public class Lift {
    protected SpeedController front;
    protected SpeedController back;
    
    /**
     * This creates a lift with Talons running the front and back rollers
     * 
     * @param frontPort the PWM port the Talon running the front polycord is attached to
     * @param backPort the PWM port the Talon running the rear polycord and stars is attached to
     */
    public Lift(int frontPort, int backPort) {
        this(frontPort, backPort, false);
    }
    
    public Lift(int frontPort, int backPort, boolean jaguars) {
        if (jaguars) {
            front = new Jaguar(frontPort);   // initalize the jaguar driving the front polycord
            back = new Jaguar(backPort); // initalize the jaguar driving the back polycord and stars
        } else {
            front = new Talon(frontPort);   // initalize the talon driving the front polycord
            back = new Talon(backPort); // initalize the talong driving the back polycord and stars
        }
    }
    
    
    /**
     * This method runs the lift up
     */
    public void lift() {
        if (front.get() != 1) { // will the lift's speed change?
            System.out.println("Lift up");
            front.set(1);   // if so, make the front pull up
            back.set(-.5);   // and make the back pull up
        }
    }
    
    /**
     * This method runs the lift down
     */
    public void reverse() {
        if (front.get() != -1) {    // will the lift's speed change?
            System.out.println("Lift reverse");
            front.set(-1);  // if so, make the front push out
            back.set(.5);    // and make the back push out
        }
    }
    
    /**
     * This method stops the lift
     */
    public void stop() {
        if (front.get() != 0) { // is the lift already stopped?
            System.out.println("Lift stop");
            front.set(0);   // if not, stop the front
            back.set(0);    // and stop the back
        }
    }
}
