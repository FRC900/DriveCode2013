package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.VM;
import edu.wpi.first.wpilibj.networktables2.stream.IOStream;
import edu.wpi.first.wpilibj.networktables2.stream.SocketConnectionServerStreamProvider;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

/**
 *
 * @author Andrew Vitkus
 */
public class PandaCom {

    private static SocketConnectionServerStreamProvider socketConnector;
    private static IOStream io;
    private static StringBuffer lastInput; //Using StringBuffer instead of String because it is thread-safe
    private static volatile boolean stop = false;
    private static volatile boolean pause = false;

    static {
        lastInput = new StringBuffer("");

        try {
            socketConnector = new SocketConnectionServerStreamProvider(1130); //setup the socket on port 1130
            io = socketConnector.accept();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    /**
     * Starts a new thread to monitor the socket for new messages
     *
     * @return is connected?
     */
    public static boolean monitor() {
        VM.addShutdownHook(new Thread() {
            public void run() {
                stop = true;
            }
        });
        
        Thread monitor = new Thread() {
            public void run() {
                while (pause) {
                    try {
                        this.sleep(100);
                    } catch (InterruptedException ex) {
                        ex.printStackTrace();
                    }
                }
                while (!stop) {
                    if (recieving()) {
                        lastInput = new StringBuffer(PandaCom.readLine());
                        try {
                            this.sleep(10); // wait 10ms then check again
                        } catch (InterruptedException ex) {
                        }
                    }
                }
            }
        };
        monitor.setPriority(Thread.MIN_PRIORITY);
        monitor.start();

        return true;
    }

    /**
     * Checks if there is a message
     *
     * @return is there a message?
     */
    public static boolean recieving() {
        try {
            return (io.getInputStream().available() != 0);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
        return false;
    }

    /**
     * Reads a line from the socket
     *
     * @return the line read from the socket
     */
    public static String readLine() {
        InputStream is = io.getInputStream();

        byte[] charBytes = new byte[2];

        StringBuffer str = new StringBuffer();
        if (recieving()) {
            lastInput.delete(0, lastInput.length());
            lastInput.setLength(1);
        }
        while (recieving()) {
            try {
                is.read(charBytes);
                lastInput.append(toChar(charBytes));
            } catch (IOException ex) {
                ex.printStackTrace();
            }

            return lastInput.toString();
        }


        return null;
    }

    /**
     * Translates a byte array into a char
     *
     * @param data an array of 2 bytes representing a char
     * @return the char coded in the byte array
     */
    private static char toChar(byte[] data) {
        if (data == null || data.length != 2) {
            return 0x0;
        }
        return (char) ((0xff & data[0]) << 8 // char is 16-bits and byte is 8 so we
                | (0xff & data[1])); // are using bitshifts to make a char from 2 bytes
    }

    /**
     * Translates a byte array into a char array
     *
     * @param data an array of bytes
     * @return an array of the chars coded for by the byte array
     */
    private static char[] toCharA(byte[] data) {
        if (data == null || data.length % 2 != 0) {
            return null;
        }
        char[] chrs = new char[data.length / 2];
        for (int i = 0; i < chrs.length; i++) {
            chrs[i] = toChar(new byte[]{ // translate pairs of bytes from data into chars
                        data[(i * 2)],
                        data[(i * 2) + 1],});
        }
        return chrs;
    }

    /**
     * Get the last line read from the socket
     *
     * @return the most recently read line
     */
    public static String getLastLine() {
        return lastInput.toString();
    }

    /**
     * Writes a String to the socket
     *
     * @param line the line to be written
     */
    public static void writeLine(String line) {
        OutputStream os = io.getOutputStream();
        try {
            os.write(lastInput.toString().getBytes());
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
    
    /**
     * Pauses the PandaBoard monitor
     */
    public static void pauseMonitor() {
        pause = true;
    }
    
    /**
     * Resumes the PandaBoard monitor
     */
    public static void resumeMonitor() {
        pause = false;
    }
}
