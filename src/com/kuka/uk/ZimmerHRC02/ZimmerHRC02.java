package com.kuka.uk.ZimmerHRC02;

import java.util.concurrent.TimeUnit;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.ZimmerHRCIOGroup;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ObserverManager;
import com.kuka.roboticsAPI.persistenceModel.processDataModel.IProcessData;
import com.kuka.task.ITaskLogger;

/**
 * @Name        ZimmerHRC02
 * @author      AdamWebb
 * @Description Class for controlling the Zimmer HRC-01/02 series gripper
 * @Modified    20/04/2022
 */
public class ZimmerHRC02 {
	
	// Control variable
	Control _ctrl;
	
	// IO Variable
	ZimmerHRCIOGroup _io;
	
	// Task Logger
	ITaskLogger _logger;
	
	// Observer manager
	ObserverManager _obsMan;
	
	// Required process data
	IProcessData _pdOpen,_pdClosed,_pdGrasped,_pdWorkpiece;
	
	/** HRCGripper constructor **/
	public ZimmerHRC02(ObserverManager obsMan,ZimmerHRCIOGroup io,ITaskLogger logger){
		try{
			
			// Declare variables
			_ctrl = new Control();
			_io = io;
			_logger = logger;
			_obsMan = obsMan;

			
			// Initialise the gripper
			Initialise();
			
			
		}catch(Exception e){_logger.error("ZimmerHRC02 | isOpen \n"+e.getMessage()+"\n");}
	}
	
	
	/** Initialise - Initial gripper setup **/
	public void Initialise(){
		try{
			
			// Initial setup
			_ctrl.toggleMotor(false);
			_ctrl.setWorkpieceNumber(0);
			ThreadUtil.milliSleep(1000);
			_ctrl.toggleMotor(true);
			
			ThreadUtil.milliSleep(1000);
			
			_ctrl.setWorkpieceNumber(1);
			
			// Check that the gripper is ready
			isReady(true);
			
		}catch(Exception e){_logger.error("ZimmerHRC02 | isOpen \n"+e.getMessage()+"\n");}
		
	}

	/** Open - open the gripper **/
	public void Open(){
		try{
			// Check gripper is ready
			if(isReady(false)){
				_ctrl.release();
			}else{_logger.error("ZimmerHRC02 | Open \n Failed to open gripper\n");}
		}catch(Exception e){_logger.error("ZimmerHRC02 | isOpen \n"+e.getMessage()+"\n");}
	}
	
	/** Close - close the gripper **/
	public void Close(){
		try{
			// Check gripper is ready
			if(isReady(false)){
				
				// Close gripper
				_ctrl.grasp();
	
				
			}else{_logger.error("ZimmerHRC02 | Close \n Failed to close gripper\n");}
		}catch(Exception e){_logger.error("ZimmerHRC02 | isOpen \n"+e.getMessage()+"\n");}
	}
	
	/** isOpen - Return whether gripper is open **/
	public boolean isOpen(){
		try{
			return _ctrl.isReleased();
		}catch(Exception e){_logger.error("ZimmerHRC02 | isOpen \n"+e.getMessage()+"\n"); return false;}
	}
	
	/** isClosed - Return whether gripper is closed **/
	public boolean isClosed(){
		try{
		  return _ctrl.isClosed();
		}catch(Exception e){_logger.error("ZimmerHRC02 | isClosed \n"+e.getMessage()+"\n");   return false;}		
	}
	
	/** isGrasped - Return whether gripper has grasped a part **/
	public boolean isGrasped(){
		try{
			return _ctrl.isGrasped();
		}catch(Exception e){_logger.error("ZimmerHRC02 | isGrasped \n"+e.getMessage()+"\n");   return false;}
	}
	
	/** waitOpen - Wait for gripper to be open, with timeout **/
	public boolean waitOpen(int nSeconds){
		try{
			
			// IO condition states whether the gripper is open
			BooleanIOCondition _pumpCond = new BooleanIOCondition(_io.getInput("IsReleased"), true);
			
			// Wait for released signal
			boolean _bSuccess=_obsMan.waitFor(_pumpCond, nSeconds, TimeUnit.SECONDS);
			
			// Print success
			if(_bSuccess){_logger.info("ZimmerHRC02 | waitOpen \n Gripper released successfully!");}
			else{_logger.error("ZimmerHRC02 | waitOpen \n Gripper released with error, or failed to release");}
			
			// Return success
			return _bSuccess;
			
		}catch(Exception e){_logger.error("ZimmerHRC02 | waitOpen\n"+e.getMessage()+"\n"); return false;}
	}
	
	/** waitOpen - Wait for gripper to be open, with timeout **/
	public boolean waitClosed(int nSeconds){
		try{
			
			// IO condition states whether the gripper is closed
			BooleanIOCondition _pumpCond = new BooleanIOCondition(_io.getInput("IsClosed"), true);
			
			// Wait for released signal
			boolean _bSuccess=_obsMan.waitFor(_pumpCond, nSeconds, TimeUnit.SECONDS);
			
			// Print success
			if(_bSuccess){_logger.info("ZimmerHRC02 | waitClosed \n Gripper closed successfully!");}
			else{_logger.error("ZimmerHRC02 | waitClosed \n Gripper closed with error, or failed to close");}
			
			// Return success
			return _bSuccess;
			
		}catch(Exception e){_logger.error("ZimmerHRC02 | waitClosed\n"+e.getMessage()+"\n"); return false;}
	}
	
	/** waitGrasped - Wait for gripper to grasp, with timeout **/
	public boolean waitGrasped(int nSeconds){
		try{
			
			// IO condition states whether the gripper is grasping
			BooleanIOCondition _pumpCond = new BooleanIOCondition(_io.getInput("IsGrasped"), true);
			
			// Wait for released signal
			boolean _bSuccess=_obsMan.waitFor(_pumpCond, nSeconds, TimeUnit.SECONDS);
			
			// Print success
			if(_bSuccess){_logger.info("ZimmerHRC02 | waitClosed \n Gripper grasped successfully!");}
			else{_logger.error("ZimmerHRC02 | waitClosed \n Gripper grasped with error, or failed to grasp");}
			
			// Return success
			return _bSuccess;
			
		}catch(Exception e){_logger.error("ZimmerHRC02 | waitGrasped\n"+e.getMessage()+"\n"); return false;}
	}
	
	/** Reset - reset gripper error state **/
	public void Reset(){
		try{
			_ctrl.reset();
		}catch(Exception e){_logger.error("ZimmerHRC02 | waitGrasped\n"+e.getMessage()+"\n");}
		
	}
	
	public void Home(){
		try
		{
			_ctrl.homing();
			_logger.info("ZimmerHRC02 | Gripper homed successfully!\n");
		}catch(Exception e){_logger.error("ZimmerHRC02 | waitGrasped\n"+e.getMessage()+"\n");}
	}
	
	/** setWorkpiece - Set required workpiece **/
	public void setWorkpiece(int nWorkpiece){
		try{
			_ctrl.setWorkpieceNumber(nWorkpiece);
			
			_logger.info("ZimmerHRC02 | setWorkpiece \n New workpiece active: " + nWorkpiece);
		}catch(Exception e){_logger.error("ZimmerHRC02 | isOpen \n"+e.getMessage()+"\n"); }
	}
	
	/** getActWorkpiece - Get the current active workpiece **/
	public int getActWorkpiece(){
		try{
			return _ctrl.getWorkpieceNumber();
		}catch(Exception e){_logger.error("ZimmerHRC02 | isOpen \n"+e.getMessage()+"\n"); return -1;}
	}
	
	/** isReady - Return whether the gripper is ready for use **/
	public boolean isReady(boolean _bLog){
		try{
			
			
			// Home gripper if required
			if(!_ctrl.isHomingOK()) Home();
			
			// Get gripper state
			boolean _bReady = _ctrl.isMotorON()&&_ctrl.isHomingOK()&&!_ctrl.inError();
			
			// Print state to console
			if(_bLog){
				if(_bReady){
					_logger.info("ZimmerHRC02 | isReady \n Gripper ready and operational\n");
				}else{
					
					_logger.error("ZimmerHRC02 | isReady \n Gripper not ready");
					if(!_ctrl.isMotorON()){_logger.error("ZimmerHRC02 | isReady \n Motor not ON"); }
					if(!_ctrl.isHomingOK()){_logger.error("ZimmerHRC02 | isReady \n Homing not OK");}
					if(_ctrl.inError()){_logger.error("ZimmerHRC02 | isReady \n SCM module in error");}

				}
			}
			
			// Return result
			return _bReady;
			
		}catch(Exception e){_logger.error("ZimmerHRC02 | isOpen \n"+e.getMessage()+"\n");    return false;}
		
	}
	
  /** Base control class containing low level methods for controlling the gripper @Modified 13/04/2022 **/
  private class Control {	

	public Control() throws Exception {}
	
	// ==========================
	//  SET
	// ==========================
	
	public void release() throws Exception{
		_io.setCmdRelease(true);
		_io.setCmdGrasp(false);
	}
	
	public void grasp() throws Exception{
		_io.setCmdRelease(false);
		_io.setCmdGrasp(true);
	}
	
	public void reset() throws Exception{
		_io.setCmdReset(true);
		ThreadUtil.milliSleep(500);
		_io.setCmdReset(false);
	}
	
	public void toggleMotor(boolean bState) throws Exception{
		_io.setCmdMotor(bState);
	}
	
	public void homing() throws Exception{
		_io.setCmdGrasp(false);
		_io.setCmdRelease(false);
		_io.setCmdHoming(true);
		ThreadUtil.milliSleep(5000);
		_io.setCmdHoming(false);
	}
	
	public void setWorkpieceNumber(int nWorkpiece) throws Exception{
		_io.setCmdGrasp(false);
		_io.setCmdRelease(false);
		_io.setCmdWorkpiece(nWorkpiece);
	}
	
	// ==========================
	//  GET
	// ==========================
	
	public boolean isReleased() throws Exception{
	  return _io.getCmdRelease();	
	}
	
	public boolean isGrasped() throws Exception{
		return _io.getCmdGrasp();
	}
	
	public boolean isClosed() throws Exception{
		return _io.getIsClosed();
	}
	

	public boolean inError() throws Exception{
		return _io.getError();
	}
	
	public boolean isMotorON() throws Exception{
	   return _io.getMotorON();
	}
	

	public boolean isHomingOK() throws Exception{
		return _io.getHomingOK();
	}
	
	public int getWorkpieceNumber() throws Exception{
		return _io.getActWorkpiece();
	}
  }
	
}
