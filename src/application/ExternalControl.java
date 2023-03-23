package application;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.generated.ioAccess.ZimmerHRCIOGroup;
import com.kuka.nav.XYTheta;
import com.kuka.nav.rel.RelativeMotion;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ObserverManager;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRAlphaRedundancy;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;
import com.kuka.uk.ZimmerHRC02.ZimmerHRC02;

/**
 * @Name        External KR Control
 * @author      Alastair Poole
 * @Description Control script for KMR platform.
 * @Modified    19/03/2023
 */
public class ExternalControl extends RoboticsAPIApplication {
	
	public static int CurrentTask = -1;
	public static int CurrentTargtPose = -1;
	public static int hashCode = -1;
	public static boolean ContinueToNext = false;
	public double linVel = 0.1, jVel = 0.3;
	public int _gripVialTray = 1,_gripVialTool = 3,  _gripVial = 4, _gripWait = 4, _velocity = 50, _highVelocity = 90 ;
	
	public static String cmdssent = "";
	public static boolean AR = false;
	
	public static double[] CartesianTarget = {0,0,0,0,0,0,0}, KMRTarget = {0,0,0},KMRSpeed = {0.1,0.1,0.1},JTarget = {0,0,0,0,0,0,0},Entries = {0,0,0,0,0,0,0};
	public static boolean NewTarget = false;
	
	public LBRAlphaRedundancy redundancy = new LBRAlphaRedundancy();
	
	public double SPMAX = 3000,SOMAX = 300;
	
	public boolean debug_ = false;
	
	private Receiver receive;
	
	
	// Zimmer gripper object
	ZimmerHRC02 _gripper;
	
	// Zimmer IO
	@Inject private ZimmerHRCIOGroup _gripperIO;
	
	@Inject 
	IApplicationData _appData;
	
	// Task Logger
	@Inject private ITaskLogger _logger;
	
	@Inject private LBR _lbr;
	
	@Inject	
	@Named("ZimmerGripper") Tool _ZimmerGripper;
	
	
	private MobileRobot _kmr;
	
	// Observer Manager
	@Inject ObserverManager _obsMan;
	
	/** initialize @Override **/
	@Override public void initialize() {
		
		LoadData _loadData = new LoadData();
		// calibrate and set externally;
		_loadData.setMass(2.14);
		_loadData.setCenterOfMass(-5.01,-7.14,47.41);
		
		_ZimmerGripper.getLoadData().fill(_loadData);
		_ZimmerGripper.attachTo(_lbr.getFlange());
		
		// Initialize zimmer gripper object
		_gripper = new ZimmerHRC02(_obsMan,_gripperIO,_logger);
		
		_gripper.Open();
		_logger.info("Starting recv thread");
		receive = new Receiver();
		
		
	}

	/** run @Override **/
	@Override public void run() {
		
		_logger.info("Starting program");
		_appData.getProcessData("taskIsFail").setValue(false);
		_appData.getProcessData("cannotComplete").setValue(false);
		_appData.getProcessData("taskIsFin").setValue(false);
		_appData.getProcessData("subTaskIsFin").setValue(false);
		while (true)
		{
			if (ContinueToNext)
			{
				_appData.getProcessData("orderHash").setValue(hashCode);
				_appData.getProcessData("taskIsFin").setValue(false);
				_appData.getProcessData("subTaskIsFin").setValue(false);
				switch (CurrentTask)
				{
				case 1:
					_logger.info("Calibrating the Camera");
					CamCalibration();
					break;
				case 2:
					_logger.info("Moving Cartesian Pose");
					for (int k=0;k<7;k++)
					{CartesianTarget[k] = Entries[k];}
					MotionBlock(0);
					break;
				case 3:
					_logger.info("Moving Cartesian Pose");
					for (int k=0;k<7;k++)
					{CartesianTarget[k] = Entries[k];}
					MotionBlock(1);
					break;
				case 4:
					_logger.info("Moving Cartesian Pose");
					for (int k=0;k<7;k++)
					{CartesianTarget[k] = Entries[k];}
					MotionBlock(2);
					break;
				case 5:
					_logger.info("Moving KMR Pose");
					for (int k=0;k<3;k++)
					{KMRTarget[k] = Entries[k];}
					RelativeMoveKMR();
					break;
				case 6:
					_logger.info("Setting tool");
					SetTool(1);
					break;
				case 7:
					_logger.info("Setting tool");
					SetTool(3);
					break;
				case 8:
					_logger.info("Setting tool");
					SetTool(4);
					break;
				case 9:
					_logger.info("Setting tool");
					for (int k=0;k<7;k++)
					{JTarget[k] = Entries[k];}
					MotionBlockJoint(0);
					break;
				case 10:
					_logger.info("Moving Joint Pose");
					for (int k=0;k<7;k++)
					{JTarget[k] = Entries[k];}
					MotionBlockJoint(1);
					break;
				case 11:
					_logger.info("Moving Joint Pose");
					for (int k=0;k<7;k++)
					{JTarget[k] = Entries[k];}
					MotionBlockJoint(2);
					break;
				case 12:
					HandleDoor();
					break;
					
				case 100:
					// exit programme
					_logger.info("Exiting the program.");
					double[] HomeJ = {-Math.PI/2,0,0,-Math.PI/2,0,Math.PI/2,0};
					MoveJ(HomeJ);
					return;
				}
				ContinueToNext = false;
				_appData.getProcessData("taskIsFin").setValue(true);
				_appData.getProcessData("subTaskIsFin").setValue(true);
			}
		}
	}
			
	public void MotionBlock(int case_)
	{
		MoveL(CartesianTarget);
		NewTarget = false;
		boolean v = _appData.getProcessData("CannotComplete").getValue();
		if (!v)
		{
			switch (case_)
			{
			case 0:
				break;
			case 1:
				_gripper.Open();
				_gripper.waitOpen(_gripWait);
				break;
			case 2:
				_gripper.Close();
				_gripper.waitClosed(_gripWait);
				break;
			}
		}
	}
	
	public void MotionBlockJoint(int case_)
	{
		//while (!NewTarget)
		//{
		//	try {
		//		Thread.sleep(5);
		//	} catch (InterruptedException e) {
		//		// TODO Auto-generated catch block
		//		e.printStackTrace();
		//	}
		//}
		MoveJ(JTarget);
		NewTarget = false;
		boolean v = _appData.getProcessData("CannotComplete").getValue();
		if (!v)
		{
			switch (case_)
			{
			case 0:
				break;
			case 1:
				_gripper.Open();
				_gripper.waitOpen(_gripWait);
				break;
			case 2:
				_gripper.Close();
				_gripper.waitClosed(_gripWait);
				break;
			}
		}
	}
	
	public void HandleDoor()
	{
		boolean ErrorOcc = false;
		// MoveJ into initial position - MoveL into starting position - Approach to feel door - force open door - move back - return to initial joint space 
		_appData.getProcessData("taskIsFin").setValue(false);
		_appData.getProcessData("subTaskIsFin").setValue(false);
		_appData.getProcessData("orderHash").setValue(hashCode);
				
		for (int k=0;k<7;k++){JTarget[k] = Entries[k];}
		MoveJ(JTarget);
		ContinueToNext = false;
		_appData.getProcessData("subTaskIsFin").setValue(true);
		
		while (!ContinueToNext)
		{
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		_appData.getProcessData("subTaskIsFin").setValue(false);
		_appData.getProcessData("orderHash").setValue(hashCode);
		for (int k=0;k<7;k++){CartesianTarget[k]=Entries[k];}
		
		MoveJ(CartesianTarget);
		ContinueToNext = false;
		
		_appData.getProcessData("subTaskIsFin").setValue(true);
		ErrorOcc = _appData.getProcessData("cannotComplete").getValue();
		
		if (!ErrorOcc)
		{
			while (!ContinueToNext)
			{
				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			_appData.getProcessData("subTaskIsFin").setValue(false);
			_appData.getProcessData("orderHash").setValue(hashCode);
			for (int k=0;k<7;k++){CartesianTarget[k]=Entries[k];}
			ContinueToNext = false;
			MoveL(CartesianTarget);
			
			_appData.getProcessData("subTaskIsFin").setValue(true);
			ErrorOcc = _appData.getProcessData("cannotComplete").getValue();
			if (!ErrorOcc)
			{
				while (!ContinueToNext)
				{
					try {
						Thread.sleep(5);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
				_appData.getProcessData("subTaskIsFin").setValue(false);
				_appData.getProcessData("orderHash").setValue(hashCode);
				for (int k=0;k<7;k++){CartesianTarget[k]=Entries[k];}
				MoveL(CartesianTarget);
				ContinueToNext = false;
				_appData.getProcessData("subTaskIsFin").setValue(true);
				ErrorOcc = _appData.getProcessData("cannotComplete").getValue();
				if (!ErrorOcc)
				{
					while (!ContinueToNext)
					{
						try {
							Thread.sleep(5);
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
					_appData.getProcessData("subTaskIsFin").setValue(false);
					_appData.getProcessData("orderHash").setValue(hashCode);
					for (int k=0;k<7;k++){JTarget[k]=Entries[k];}
					MoveJ(JTarget);
						
					_appData.getProcessData("subTaskIsFin").setValue(true);
					_appData.getProcessData("taskIsFin").setValue(true);
				}
			}	
		}
	}
	
	public void CamCalibration()
	{
		_appData.getProcessData("taskIsFin").setValue(false);
		double[][] JointPosns={{-105.06, 22.24,8.14,-73.72,7.97,84.75,-6.39},
				{-151.19, 38.22, 5.9, -73.71, 9.78, 75.89, 30.09},
				{-97.38, 84.87, -11.39, -33.29, -3.17, 119.90, 162.23},
				{-46.02, 40.82, -11.39, -62.64, -21.01, 118.04, 162.23},
				{0.01, 9.29, -51.51, -63.23, -21.00, 108.53, 68.77},
				{-88.46, -30.58, -52.18, -83.16, 13.06, 107.84, -24.24},
				{-157.24, 68.07, -5.10, -38.53, 43.48, 117.48, -51},
				{-129.07, 47.66, -5.11, -42.56, 20.82, 103.28, -22.90},
				{-65.24, 43.24, -5.12,-48.94, -2.81,91.44,27.19},
				{-64.58, -3.46,-5.12,-51.21,-2.61,109.44,26.22}};
		for (int k=0;k<10;k++)
		{
			
			while (!ContinueToNext)
			{
				_appData.getProcessData("orderHash").setValue(hashCode);
				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			_logger.info("Moving to joint position: " + Integer.toString(k+1) + " of 10.");
			_appData.getProcessData("subTaskIsFin").setValue(false);
			double[] Jnow = JointPosns[k];
			for (int j=0;j<7;j++){Jnow[j]*=Math.PI/180;}
			try{MoveJ(Jnow);}
			catch(Exception e)
			{
				_appData.getProcessData("taskIsFail").setValue(false);
			}
			ContinueToNext = false;
			_appData.getProcessData("orderHash").setValue(hashCode);
		}
		_appData.getProcessData("taskIsFin").setValue(true);
		AR = false;
	}
	
	public void RelativeMoveKMR()
	{
		try{
			_kmr.lock();
			RelativeMotion _relMotion = new RelativeMotion(KMRTarget[0],KMRTarget[1],KMRTarget[2]);
			_relMotion.setVelocity(new XYTheta(KMRSpeed[0],KMRSpeed[1],KMRSpeed[2]));
			_kmr.execute(_relMotion);
			_appData.getProcessData("subTaskIsFin").setValue(true);
			_appData.getProcessData("taskIsFin").setValue(true);
		}
		catch(Exception e)
		{
			_logger.info("Error:"+e.toString());
			_appData.getProcessData("cannotComplete").setValue(true);
		}
		_kmr.unlock();
	}
	
	public void SetTool(int wp)
	{
		_gripper.setWorkpiece(wp);
		_appData.getProcessData("subTaskIsFin").setValue(true);
	}
	
	public void MoveJ(double[] Angles)
	{
		_ZimmerGripper.move(ptp(Angles).setJointVelocityRel(jVel));
		_appData.getProcessData("subTaskIsFin").setValue(true);
	}
	
	public void MoveL(double[] P)
	{
		redundancy.setAlpha(P[6]);
		Frame cp_ = _lbr.getCurrentCartesianPosition(_ZimmerGripper.getDefaultMotionFrame()),target_;
		target_= cp_;
		target_.setX(P[0]);
		target_.setY(P[1]);
		target_.setZ(P[2]);
		target_.setAlphaRad(P[3]);
		target_.setBetaRad(P[4]);
		target_.setGammaRad(P[5]);
		target_.setRedundancyInformation(_lbr, redundancy);
		
		if (cp_.hasPathTo(target_))
		{
			_ZimmerGripper.move(ptp(target_).setJointVelocityRel(jVel));//.setCartVelocity(linVel));
			_appData.getProcessData("subTaskIsFin").setValue(true);
			return;
		}
		_appData.getProcessData("cannotComplete").setValue(true);
	}

	public void ApproachF(double[] P)
	{
		double flim = 5;
		Frame startingPose=_lbr.getCurrentCartesianPosition(_ZimmerGripper.getDefaultMotionFrame()),eP;
		eP = startingPose.copyWithRedundancy();
		eP.setX(P[0]);eP.setY(P[1]);eP.setZ(P[2]);
		eP.setAlphaRad(P[3]);eP.setBetaRad(P[4]);eP.setGammaRad(P[5]);
		
		if (startingPose.hasPathTo(eP))
		{
			_appData.getProcessData("cannotComplete").setValue(true);
			return;
		}
		double x0=startingPose.getX(),y0=startingPose.getY(),z0=startingPose.getZ();
		double fx0 = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getX(), fy0 = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getY(),fz0 = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getZ();
		double fxt = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getX(), fyt = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getY(),fzt = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getZ();
		double df = Math.abs( (P[0] - x0)*(fx0 - fxt) + (P[1] - y0)*(fy0 - fyt) + (P[2] - z0)*(fz0 - fzt))/Math.sqrt( (P[0] - x0)*(P[0] - x0) + (P[1] - y0)*(P[1] - y0) + (P[2] - z0)*(P[2] - z0));
		
		CartesianImpedanceControlMode cartImp = createCartImpl(SPMAX,SPMAX,SPMAX,SOMAX,SOMAX,SOMAX,0,0,0,5,50,10,0.1,0.1,0.1);
		IMotionContainer motionContainer = _lbr.getFlange().moveAsync(lin(eP).setMode(cartImp));
		while (df<flim && motionContainer.isFinished())
		{
			fxt = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getX(); fyt = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getY();fzt = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getZ();
			fxt = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getX(); fyt = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getY();fzt = _lbr.getExternalForceTorque(_ZimmerGripper.getDefaultMotionFrame()).getForce().getZ();
			df = Math.abs( (P[0] - x0)*(fx0 - fxt) + (P[1] - y0)*(fy0 - fyt) + (P[2] - z0)*(fz0 - fzt))/Math.sqrt( (P[0] - x0)*(P[0] - x0) + (P[1] - y0)*(P[1] - y0) + (P[2] - z0)*(P[2] - z0));
		}
		if (!motionContainer.isFinished())
		{
			motionContainer.cancel();
			_appData.getProcessData("subTaskIsFin").setValue(true);
			return;
		}
		_appData.getProcessData("cannotComplete").setValue(true);
	}
	
	public void MoveF(double[] P,double Fdom)
	{
		Frame startingPose=_lbr.getCurrentCartesianPosition(_ZimmerGripper.getDefaultMotionFrame()),eP;
		eP = startingPose.copyWithRedundancy();
		eP.setX(P[0]);eP.setY(P[1]);eP.setZ(P[2]);
		eP.setAlphaRad(P[3]);eP.setBetaRad(P[4]);eP.setGammaRad(P[5]);
		redundancy.setAlpha(P[6]);
		eP.setRedundancyInformation(_lbr, redundancy);
		
		if (startingPose.hasPathTo(eP))
		{
			_appData.getProcessData("cannotComplete").setValue(true);
			return;
		}
		
		double dx = P[0] - startingPose.getX();
		double dy = P[0] - startingPose.getX();
		double dz = P[0] - startingPose.getX();
		double da = Math.sqrt(dx*dx + dy*dy+dz*dz);
		if (da>0)
		{
			CartesianImpedanceControlMode cartImp = createCartImpl(SPMAX,SPMAX,SPMAX,SOMAX,SOMAX,SOMAX,0,0,0,10,50,10,0.01,0.01,0.01);
			IMotionContainer motionContainer = _lbr.getFlange().move(lin(eP).setMode(cartImp));
			if (motionContainer.hasError())
			{
				_appData.getProcessData("cannotComplete").setValue(true);
				return;
			}
			_appData.getProcessData("subTaskIsFin").setValue(true);
			return;
		}
		_appData.getProcessData("cannotComplete").setValue(true);
	}
	
	public boolean CheckMove(Frame target_)
	{
		try{
			_lbr.getInverseKinematicFromFrameAndRedundancy(target_);
			return true;
		}
		catch(Exception e)
		{
			_appData.getProcessData("cannotComplete").setValue(false);
			return false;}
	}
	
	protected static CartesianImpedanceControlMode createCartImpl(double sX,double sY,double sZ,double sA,double sB,double sC,double constForceX,double constForceY,double constForceZ,double maxDx, double maxDy,double maxDz,double maxDa,double maxDb,double maxDc)
    {
        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
        cartImp.parametrize(CartDOF.X).setStiffness(sX);
        cartImp.parametrize(CartDOF.Y).setStiffness(sY);
        cartImp.parametrize(CartDOF.Z).setStiffness(sZ);
        cartImp.parametrize(CartDOF.A).setStiffness(sA);
        cartImp.parametrize(CartDOF.B).setStiffness(sB);
        cartImp.parametrize(CartDOF.C).setStiffness(sC);
        
        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(constForceX);
        cartImp.parametrize(CartDOF.Y).setAdditionalControlForce(constForceY);
        cartImp.parametrize(CartDOF.Z).setAdditionalControlForce(constForceZ);

        cartImp.setMaxPathDeviation(maxDx, maxDy, maxDz, maxDa, maxDb, maxDc);
        return cartImp;
    }  
	
}