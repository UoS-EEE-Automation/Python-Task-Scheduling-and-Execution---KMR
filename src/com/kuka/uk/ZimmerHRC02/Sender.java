package com.kuka.uk.ZimmerHRC02;


import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.geometry.Frame;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.nav.robot.MobileRobotManager;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;

/**
 * Author:C.Loukas (SEARCH Lab)
 * charalampos.loukas@strath.ac.uk
 * Implementation of a cyclic background task.
 * Send Data to LabVIEW from KMR
 */
public class Sender extends RoboticsAPICyclicBackgroundTask {
	@Inject
	private LBR _lbr;
	
	@Inject	
	@Named("ZimmerGripper") Tool _ZimmerGripper;
	
	@Inject 
	IApplicationData _appData;
	
	//kmr data
	@Inject private MobileRobotManager _robotManager;
	private MobileRobot _kmr;
	
	
	int port=30001;
	byte[] ExternalMachineIP={(byte) 172,31,1,(byte) 137}; 
	InetAddress address;
	
	@Override
	public void initialize() {
		initializeCyclic(0, 100, TimeUnit.MILLISECONDS,CycleBehavior.BestEffort);
		
		_ZimmerGripper.attachTo(_lbr.getFlange());
		int instanceId =1; // lookup in the StationSetup from the KMR in Configuration Tab.
		_kmr = _robotManager.getRobot(instanceId);
	}

	@Override
	public void runCyclic() {
		try {
	        
		address = InetAddress.getByAddress(ExternalMachineIP);
		
		double j0 = _lbr.getCurrentJointPosition().get(0);
		double j1 = _lbr.getCurrentJointPosition().get(1);
		double j2 = _lbr.getCurrentJointPosition().get(2);
		double j3 = _lbr.getCurrentJointPosition().get(3);
		double j4 = _lbr.getCurrentJointPosition().get(4);
		double j5 = _lbr.getCurrentJointPosition().get(5);
		double j6 = _lbr.getCurrentJointPosition().get(6);
		double _x=_lbr.getCurrentCartesianPosition(_ZimmerGripper.getDefaultMotionFrame()).getX();
		double _y=_lbr.getCurrentCartesianPosition(_ZimmerGripper.getDefaultMotionFrame()).getY();
		double _z=_lbr.getCurrentCartesianPosition(_ZimmerGripper.getDefaultMotionFrame()).getZ();
		double _a=_lbr.getCurrentCartesianPosition(_ZimmerGripper.getDefaultMotionFrame()).getAlphaRad();
		double _b=_lbr.getCurrentCartesianPosition(_ZimmerGripper.getDefaultMotionFrame()).getBetaRad();
		double _c=_lbr.getCurrentCartesianPosition(_ZimmerGripper.getDefaultMotionFrame()).getGammaRad();
		
		boolean isFin = _appData.getProcessData("taskIsFin").getValue();
		boolean subIsFin = _appData.getProcessData("subTaskIsFin").getValue();
		boolean isFail = _appData.getProcessData("taskIsFail").getValue();
		boolean subIsImp = _appData.getProcessData("cannotComplete").getValue();
		
		
		double alpha_a = _lbr.getAlphaAngle(_lbr.getCurrentJointPosition());
		double kmr_pos_x=_kmr.getPose().getX();
		double kmr_pos_y=_kmr.getPose().getY();
		double kmr_pos_theta=_kmr.getPose().getTheta();
		double _battery=(_kmr.getBatteryState())*100;
		boolean _isCharging=(Boolean)_kmr.isBatteryCharging();//battery status return
		
		int orderH = _appData.getProcessData("orderHash").getValue();
		
		String res=String.valueOf(j0)+" "+String.valueOf(j1)+" "+String.valueOf(j2)+" "+String.valueOf(j3)+" "+String.valueOf(j4)+" "+String.valueOf(j5)+" "+String.valueOf(j6)+" "+String.valueOf(_x)+" "+String.valueOf(_y)+" "+String.valueOf(_z)
				+" "+String.valueOf(_a)+" "+String.valueOf(_b)+" "
				+String.valueOf(_c)+" "+String.valueOf(alpha_a)+" "+String.valueOf(kmr_pos_x)+" "+
				String.valueOf(kmr_pos_y)+" "+String.valueOf(kmr_pos_theta)+" "+String.valueOf(_battery)+" "+String.valueOf(_isCharging)+" "+String.valueOf(isFin) + " "+String.valueOf(subIsFin)+" "+String.valueOf(isFail)+" "+String.valueOf(subIsImp)+" "+String.valueOf(orderH);
		// joint - pose - kmr pos - is finished - subtask is finished - failed - sub task is impossible
		
		byte[] message = res.getBytes();
		
		DatagramPacket packet = new DatagramPacket(message, message.length, address, port);
		DatagramSocket dSocket;
		dSocket = new DatagramSocket();
		dSocket.send(packet);
		
		}catch (UnknownHostException e) {  e.printStackTrace(); }
		 catch (IOException e) { e.printStackTrace(); }
	}
}