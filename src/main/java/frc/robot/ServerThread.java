package frc.robot;

// Network sockets

import java.io.*;

import java.net.*;

import edu.wpi.first.wpilibj.Timer;

// thread to communicate with pi

public class ServerThread extends Thread
{
  private Socket socket;

  /*
   * 
   * By declaring these variables as public static, I can access them from any
   * class
   * 
   * by referencing them as ServerThread.varname. This way I can process the data
   * 
   * externally, if I want
   * 
   */

  public static double xCentroid = 0;
  public static double yCentroid = 0;
  public static double angleToTarget = 0;
  public static int numObjects = 0;
  public static boolean signal = false;

  public ServerThread(Socket socket)
  {
    this.socket = socket;
  }

  public void run()
  {
    try {
      InputStream input = socket.getInputStream();
      BufferedReader reader = new BufferedReader(new InputStreamReader(input));
      OutputStream output = socket.getOutputStream();
      PrintWriter writer = new PrintWriter(output, true);
      String text;
      String s = "";

      // Socket connection extablished, all RPi conversation occurs in the do loop
      do {
        if (signal)
        {
          signal = false;
          writer.println("bye");
          Timer.delay(0.15);
          break;
        }
        // Let's ask for number of objects
        writer.println("pos:");
        Timer.delay(0.1);
        text = reader.readLine();
        //System.out.println("Server read: " + text);
        if (text.contains("X")) {
          yCentroid = Double.parseDouble(text.substring(text.indexOf(":") + 1, text.indexOf("+")));
          xCentroid = Double.parseDouble(text.substring(text.indexOf(";") + 1, text.indexOf(",")));
        }
      } while (true);
      socket.close();
      System.out.println("Socket has closed, thread will end");
    } catch (IOException ex) {
      System.out.println("Server exception:" + ex.getMessage());
      ex.printStackTrace();
    }
    catch (NullPointerException ex) {
      System.out.println("Null pointer exception:" + ex.getMessage());
      ex.printStackTrace();
    }
  }
}