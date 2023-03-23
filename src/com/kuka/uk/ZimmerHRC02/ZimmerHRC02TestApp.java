package com.kuka.uk.ZimmerHRC02;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import javax.inject.Inject;

import com.kuka.generated.ioAccess.ZimmerHRCIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ObserverManager;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.task.ITaskLogger;

/**
 * @Name        ZimmerHRC02TestApp
 * @author      AdamWebb
 * @Description Test app for Zimmer HRC-01/02 series gripper
 * @Modified    20/04/2022
 */
public class ZimmerHRC02TestApp extends RoboticsAPIApplication {
	
	// Zimmer gripper object
	ZimmerHRC02 _gripper;
	
	// Zimmer IO
	@Inject private ZimmerHRCIOGroup _gripperIO;
	
	// Application UI
	@Inject private IApplicationUI _appUI;
	
	// Task Logger
	@Inject private ITaskLogger _logger;
	
	@Inject private LBR _lbr;
	
	// Observer Manager
	@Inject ObserverManager _obsMan;
	
	/** initialize @Override **/
	@Override public void initialize() {
		
		// Initialize zimmer gripper object
		_gripper = new ZimmerHRC02(_obsMan,_gripperIO,_logger);
		
		_gripper.Open();
	}

	/** run @Override **/
	@Override public void run() {
		
		// Option
		int _nOption=-1;
		
		// Check option
		while(_nOption!=5){
			
			// Display options
			_nOption=_appUI.displayModalDialog(ApplicationDialogType.QUESTION,
					"Zimmer HRC02 Test Application \n Select an option..", 
					"Open","Close","Set Workpiece","PickPart","Homing","Exit");
			
			// Decide on option
			switch(_nOption){
				case 0: _gripper.Open(); _gripper.waitOpen(10); break; // Open
				case 1: _gripper.Close(); _gripper.waitClosed(10); break; // Close
				case 2: setWorkpieceNumber(); break; // setWorkpieceNumber()
				case 3: pickPart();           break; // Pick Part
				case 4: _gripper.Home(); break; // Home gripper
				case 5: break; // Exit
			}	
		}
		
	}
	
	/** Pick part **/
	private void pickPart(){
		
		// Initialise
		
		_gripper.setWorkpiece(1);
		
		_gripper.Open();
		
		_gripper.waitOpen(10);
		
		// --
		
		
		// Pick
		
		_lbr.move(ptp(getFrame("/Pick/Pre")).setJointVelocityRel(0.5));
		
		_lbr.move(lin(getFrame("/Pick")).setCartVelocity(150));
		
		_gripper.Close();
		
		_gripper.waitGrasped(10);
		
		_lbr.move(lin(getFrame("/Pick/Post")).setCartVelocity(150));
		
		// --
		
		// Place
		
		_lbr.move(ptp(getFrame("/Place/Pre")).setJointVelocityRel(0.5));
		
		_lbr.move(lin(getFrame("/Place")).setCartVelocity(150));
		
		_gripper.Open();
		
		_gripper.waitOpen(10);
		
		_lbr.move(lin(getFrame("/Place/Post")).setCartVelocity(150));
		
		// --
		
		
	}
	
	/** setWorkpieceNumber **/
	private void setWorkpieceNumber(){
		
	    int _nOption = _appUI.displayModalDialog(ApplicationDialogType.QUESTION,"Select Required Workpiece", 
	    	"1","2","3","4","5","6","7","8","9","10","..");
	    
	    if(_nOption==10){
		    _nOption = _appUI.displayModalDialog(ApplicationDialogType.QUESTION,"Select Required Workpiece", 
			    	"11","12","13","14","15");
		    
		    _gripper.setWorkpiece(10+(_nOption+1));
		    _logger.info("ZimmerHRC02TestApp | setWorkpieceNumber \n Chosen workpiece: " + (10+ (_nOption+1)));
		    
	    }else{
	    	_gripper.setWorkpiece((_nOption+1));
	    	_logger.info("ZimmerHRC02TestApp | setWorkpieceNumber \n Chosen workpiece: " + (_nOption+1));
	    }
	    
	    
	    
	  
		
	}
}