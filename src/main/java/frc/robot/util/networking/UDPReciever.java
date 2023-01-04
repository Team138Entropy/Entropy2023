package frc.robot.util.networking;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;

public class UDPReciever {
    DatagramSocket recieveSocket = null;
    byte[] receiveData = new byte[2048];
    DatagramPacket recievePacket = null;
  
    /**
     * Constructor for the UDP reciever. Sets up internal memory structures in prep to start listening
     * for packets.
     *
     * @param listen_to_addr String of the IP address of the coprocessor (For example, "10.17.36.8")
     * @param listen_on_port integer port number to listen on. Often between 5800 and 5810 per FMS
     *     whitepaper. Must match whatever port the coprocessor is sending information to.
     */
    public UDPReciever(String listen_from_addr_in, int listen_on_port_in) {
      try {
        recieveSocket = new DatagramSocket(listen_on_port_in);
        recievePacket = new DatagramPacket(receiveData, receiveData.length);
        recieveSocket.setSoTimeout(20);
      } catch (IOException e) {
        System.out.println("Error: Cannot set up UDP reciever socket: " + e.getMessage());
        recieveSocket = null;
      }
    }
  
    /** Listens for all packets on the connection.. passes to the parser in a thread pool */
    public String getPacket() {
      if (recieveSocket != null) {
        try {
          recieveSocket.receive(recievePacket);
          String rx_string = new String(recievePacket.getData(), 0, recievePacket.getLength());
          return rx_string;
        } catch (IOException e) {
            //System.out.println("IOExexception on getPacket");
            //System.out.println(e);
        }
      }
      return "";
    }
  }