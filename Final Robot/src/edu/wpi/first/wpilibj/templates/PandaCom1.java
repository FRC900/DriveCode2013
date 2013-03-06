package edu.wpi.first.wpilibj.templates;

import com.sun.cldc.jna.ptr.IntByReference;
import com.sun.squawk.io.j2me.socket.Protocol;
import com.sun.squawk.platform.posix.natives.Socket;
//import com.sun.squawk.platform.posix.natives.SocketImpl;
import com.sun.squawk.platform.posix.vxworks.natives.SocketImpl;
import edu.wpi.first.wpilibj.networktables2.stream.SocketConnectionStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

/**
 *
 * @author Andrew Vitkus
 */
public class PandaCom1 {
    
    private Protocol socketProt;
    private SocketImpl socket;
    private int sockFD;
    private DataInputStream dis;
    private DataOutputStream dos;
    private StringBuffer lastInput; //Using StringBuffer instead of String because it is thread-safe
    private volatile boolean pause = false;
    private volatile boolean stop = false;
    
    PandaCom1() {
        lastInput = new StringBuffer("");

        //startConnection();
    }
    
    private boolean startConnection() {
        try {
            IntByReference addr = new IntByReference(0);
            
            socket.inet_pton("010.009.000.040", addr);
            
            Socket.sockaddr_in sockaddr_in = new Socket.sockaddr_in();
            sockaddr_in.sin_family = Socket.AF_INET;
            sockaddr_in.sin_addr = addr.getValue();
            sockaddr_in.sin_port = 1130;
            sockaddr_in.sin_len = sockaddr_in.size();
            
            socket = new SocketImpl();
            
            sockFD = socket.socket(Socket.AF_INET, Socket.SOCK_STREAM, 0);
            socket.bind(sockFD, sockaddr_in, sockaddr_in.size());
            socket.connect(sockFD, sockaddr_in, sockaddr_in.size());
            socket.listen(sockFD, 1);
            
            socketProt = new Protocol(sockFD);
            
            dis = socketProt.openDataInputStream();
            dos = socketProt.openDataOutputStream();
            
            System.out.println("Connection Successful!");
            
            return true;
        } catch (IOException ex) {
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
    public boolean recieving() {
        try {
            return (dis.available() > 0);
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
    public String readLine() {
        StringBuffer str = new StringBuffer();
        if (recieving()) {
            lastInput = null;
        }
        while (recieving()) {
            try {
                str.append(dis.readUTF());
                System.out.print(str);
                lastInput = str;
            } catch (IOException ex) {
                ex.printStackTrace();
            }
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
            dos.writeUTF(line);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
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
                    /*try{
                     SocketConnectionStream s = new SocketConnectionStream("10.9.0.40", 1130);
                     System.out.println("Connected!");
                     Thread.sleep(500);
                     } catch (InterruptedException ex) {
                     System.out.println("THREAADDDDDDD ERRRRORRRRRRRRRRRR");
                     } catch(IOException e) {
                     System.out.println("Failllllllllllllllllllllll");
                        
                     }
                     */
                    if (startConnection()) {
                        boolean timedOut = false;
                        long time = System.currentTimeMillis();
                        while (!recieving() && !timedOut) {
                            timedOut = System.currentTimeMillis() - time > 500;
                            try {
                                System.out.println("Feed me");
                                Thread.sleep(50);
                            } catch (InterruptedException ex) {
                                ex.printStackTrace();
                            }
                        }
                        if (!timedOut) {
                            lastInput = new StringBuffer(readLine());
                            //System.out.println(lastInput);
                        } else {
                            System.out.println("Timed out");
                        }
                    }
                    try {
                        System.out.println("Waiting...");
                        Thread.sleep(500);
                        System.out.println("Done waiting!");
                    } catch (InterruptedException ex) {
                        ex.printStackTrace();
                    }
                }
            }
            System.out.println("Why is this done?");
        }
    }
}
