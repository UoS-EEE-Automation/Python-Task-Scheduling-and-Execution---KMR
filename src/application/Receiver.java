package application;


import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.IApplicationData;


/**
 * Author: A.Poole, C.Loukas
 * Email: alastair.poole@strath.ac.uk, charalampous.loukas@....
 * 
 * thread receiving move commands from UDP socket.
 */

class Receiver implements Runnable {
	DatagramSocket ss;
	int ControlMode;
	int _port, _timeOut;
	boolean NotStandIn = true;
	String[] variables;
	@Inject 
	IApplicationData _appData;
	Receiver(){
		_port = 30010;
		_timeOut = 10*1000;
		Thread t= new Thread(this);
		t.setDaemon(true);
		t.start();
	}
	public void run() {
		byte[] buf=new byte[256];
		try{
		ss= new DatagramSocket(_port);
		ss.setSoTimeout(_timeOut);
		}
		catch (Exception e)
		{
			e.printStackTrace();
			try {
				ss= new DatagramSocket(_port);
				ss.setSoTimeout(_timeOut);
			} catch (SocketException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
				return;
			}
		}
		while(true)
		{
			try{
				DatagramPacket receivedPacket=new DatagramPacket(buf, buf.length);
				ss.receive(receivedPacket);
				String received=new String(receivedPacket.getData(),0,receivedPacket.getLength());
				
				ExternalControl.cmdssent = received;
				
				variables = received.split(" ");
				
				ExternalControl.CurrentTask = Integer.parseInt(variables[0]);
				if (ExternalControl.CurrentTask==100)
				{// kill thread command
					ss.close();
					return;
				}
				
				// Make boolean read-ins agnostic to string encoding of booleans on the other side 
				NotStandIn = variables[1].startsWith("T") || variables[1].startsWith("t");
				
				if (NotStandIn)
				{
					ExternalControl.AR = true;
					ExternalControl.ContinueToNext = variables[2].startsWith("T") || variables[2].startsWith("t");
					for (int k=0;k<7;k++)
					{
						ExternalControl.Entries[k] = Double.parseDouble(variables[k+3]);
					}
					ExternalControl.hashCode = Integer.parseInt(variables[10]);
					// setting app data vars in receiver didn't work - needs investigation.
					//_appData.getProcessData("orderHash").setValue(Integer.parseInt(variables[10]));					
				}
			}
			catch(Exception e)
			{
				try {
					Thread.sleep(5);
				} catch (InterruptedException e1) {
					e1.printStackTrace();
					ss.close();
					return;
				}
			}			
		}
	}
	
	public void Kill()
	{
		// kill the thread - essential for re-using the port without having to undergo complete restart.
		ss.close();
		return;
	}
	
}


