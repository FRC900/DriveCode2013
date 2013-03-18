package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.platform.posix.GCFSocketsImpl;
import edu.wpi.first.wpilibj.networktables2.stream.SocketConnectionStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import javax.microedition.io.Connector;
import javax.microedition.io.SocketConnection;

/**
 *
 * @author Andrew Vitkus
 */
public class PandaCom {

    private SocketConnectionStream scs;
    private GCFSocketsImpl socket;
    private InputStreamReader isr;
    private OutputStreamWriter osw;
    private StringBuffer lastInput; //Using StringBuffer instead of String because it is thread-safe
    private volatile boolean pause = false;
    private boolean stop = false;

    PandaCom() {
        lastInput = new StringBuffer("");

        //startConnection();
    }

    private boolean startConnection() {
        try {
            if (scs != null) {
                scs.close();
                scs = null;
            }

            scs = new SocketConnectionStream("10.9.0.40", 1130);

            //SocketConnection socketConnection = (SocketConnection) Connector.open("socket://10.9.0.40:1130");

            InputStream is = scs.getInputStream();
            OutputStream os = scs.getOutputStream();

            if (isr != null) {
                //try {
                isr.close();
                osw.close();
                /*} catch (IOException ex) {
                 ex.printStackTrace();
                 }*/
            }



            isr = new InputStreamReader(is);
            osw = new OutputStreamWriter(os);
            //socketConnection.close();

            //System.out.println("Connection Successful!");
            return true;
        } catch (IOException ex) {
            /*if (scs != null) {
             scs.close();
             }*/
            scs = null;

            try {
                socket = new GCFSocketsImpl();
                socket.close(socket.open("10.9.0.40", 1130, SocketConnection.LINGER));
            } catch (IOException ex1) {
                ex1.printStackTrace();
            } finally {
                socket = null;
            }

            System.out.println("Connection Failed!");
            ex.printStackTrace();

            return false;
        }
    }

    /**
     * Starts a new thread to monitor the socket for new messages
     *
     * @return is connected?
     */
    public boolean monitor() {
        MonitorThread monitor = new MonitorThread();
        monitor.start();

        return true;
    }

    /**
     * Checks if there is a message
     *
     * @return is there a message?
     */
    public boolean recieving() throws IOException {
        return (isr.ready());
    }

    /**
     * Reads a line from the socket
     *
     * @return the line read from the socket
     * @throws IOException
     */
    public String readLine() throws IOException {
        StringBuffer str = new StringBuffer();
        if (recieving()) {
            lastInput = null;
        }
        while (recieving()) {
            int in = isr.read();
            System.out.print(in + " ");
            char c = (char) in;
            //System.out.print(c);
            str.append(c);
            lastInput = str;
        }
        System.out.println();

        if (lastInput != null) {
            return lastInput.toString();
        } else {
            return null;
        }
    }

    /**
     * Translates a byte array into a char
     *
     * @param data an array of 2 bytes representing a char
     * @return the char coded in the byte array
     */
    private char toChar(byte[] data) {
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
    private char[] toCharA(byte[] data) {
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
    public String getLastLine() {
        return lastInput.toString();
    }

    /**
     * Writes a String to the socket
     *
     * @param line the line to be written
     */
    public void writeLine(String line) {
        try {
            osw.write(line, 0, line.length());
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    public String requestLine() throws IOException {
        startConnection();
        return readLine();
    }

    /**
     * Pauses the PandaBoard monitor
     */
    public void pauseMonitor() {
        pause = true;
    }

    /**
     * Resumes the PandaBoard monitor
     */
    public void resumeMonitor() {
        pause = false;
    }

    private class MonitorThread extends Thread {

        public void run() {
            while (!stop) {
                if (pause) {
                    System.out.println("There be dragons...and paused threads...");
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException ex) {
                        ex.printStackTrace();
                    }
                } else {
                    if (startConnection()) {
                        try {
                            boolean timedOut = false;
                            long time = System.currentTimeMillis();
                            while (!recieving() && !timedOut) {
                                timedOut = System.currentTimeMillis() - time > 500;
                                try {
                                    //System.out.println("Feed me");
                                    Thread.sleep(50);
                                } catch (InterruptedException ex) {
                                    ex.printStackTrace();
                                }
                            }
                            if (!timedOut) {
                                lastInput = new StringBuffer(readLine());
                                System.out.println(lastInput);
                            }
                        } catch (IOException ex) {
                            ex.printStackTrace();
                        }
                    }
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException ex) {
                        ex.printStackTrace();
                    }
                }
            }
        }
    }
}