package com.kuka.uk.ZimmerHRC02;


import java.util.concurrent.TimeUnit;
import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.generated.ioAccess.ZimmerHRCIOGroup;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.conditionModel.ObserverManager;
import com.kuka.roboticsAPI.persistenceModel.processDataModel.IProcessData;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;
import com.kuka.task.ITaskLogger;

/**
 * @Name        ZimmerHRC02BackgroundTask
 * @author      AdamWebb
 * @Description Background task for controlling the Zimmer HRC-01/02 series gripper
 *              Adds user keys for operating the gripper
 * @Modified    20/04/2022
 */
public class ZimmerHRC02BackgroundTask extends RoboticsAPICyclicBackgroundTask {

	// Zimmer gripper variable
	private ZimmerHRC02 _gripper;
	
	// Zimmer IO Group
	@Inject ZimmerHRCIOGroup _gripperIO;
	
	// Media flange IO group
	@Inject MediaFlangeIOGroup _mfIO;
	
	// User keys
	IUserKeyBar _keyBar;
	IUserKey _open,_close,_wpPlus,_wpMinus;
	IUserKeyListener _openListener,_closeListener,_wpPlusListener,_wpMinusListener;
	
	// Application UI
	@Inject IApplicationUI _appUI;
	
	// Application Data
	@Inject IApplicationData _appData;
	
	// Process data
	IProcessData _pdOpen,_pdClosed,_pdGrasped,_pdWorkpiece;
	
	// Task Logger
	@Inject ITaskLogger _logger;
	
	// Observer Manager
	@Inject ObserverManager _obsMan;
	
	// Flash bit
	boolean _bFlash,_bFlashMF = false;
	
	// Workpiece number
	public int nNewWorkpiece;
	
	public String wpPlusText;

	/** initialize @Override **/
	@Override public void initialize() {
		
		// initialize your task here
		initializeCyclic(0,100, TimeUnit.MILLISECONDS,
				CycleBehavior.BestEffort);
		
		
		// Initialise zimmer gripper object
		_gripper = new ZimmerHRC02(_obsMan,_gripperIO,_logger);
		
		nNewWorkpiece = 1;
		
		// Declare process data
		_pdOpen = _appData.getProcessData("gripIsOpen");
		_pdClosed = _appData.getProcessData("gripIsClosed");
		_pdGrasped = _appData.getProcessData("gripIsGrasped");
		_pdWorkpiece = _appData.getProcessData("gripCurWorkpiece");
		
		// Create user key bar
		_keyBar = _appUI.createUserKeyBar("Zimmer");
		
		// OPEN key listener
		_openListener = new IUserKeyListener() {
			
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				// TODO Auto-generated method stub
				_gripper.Open();
				_gripper.waitOpen(10);
				setLEDState();
			}
		};
		
		// CLOSE key listener
		_closeListener = new IUserKeyListener() {
			
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				// TODO Auto-generated method stub
				_gripper.Close();
				_gripper.waitClosed(10);
				setLEDState();
			}
		};
		
		// WP Plus key listener
		_wpPlusListener = new IUserKeyListener() {
			
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				// TODO Auto-generated method stub
				
				if(nNewWorkpiece < 15){
	
					nNewWorkpiece = nNewWorkpiece + 1;
				
					
					_gripper.setWorkpiece(nNewWorkpiece);
					
					// WP Plus text
					_wpPlus.setText(UserKeyAlignment.Middle, "+ " + nNewWorkpiece);
					
					ThreadUtil.milliSleep(1000);
				}

			}
		};
		
		// WP Minus key listener
		_wpMinusListener = new IUserKeyListener() {
			
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				// TODO Auto-generated method stub
				
				if(nNewWorkpiece > 0){
					nNewWorkpiece = nNewWorkpiece - 1;
					
					
					_gripper.setWorkpiece(nNewWorkpiece);
					
					// WP Plus text
					_wpPlus.setText(UserKeyAlignment.Middle, "+ " + nNewWorkpiece);
					
					ThreadUtil.milliSleep(1000);
					
				}

			}
		};
		
		// Set OPEN key text, and add listener
		_open = _keyBar.addUserKey(0, _openListener,false);
		_open.setText(UserKeyAlignment.TopLeft, "Open");
		_open.setLED(UserKeyAlignment.BottomRight, UserKeyLED.Red, UserKeyLEDSize.Normal);
		
		// Set CLOSE key text, and add listener
		_close = _keyBar.addUserKey(1, _closeListener,false);
		_close.setText(UserKeyAlignment.TopLeft, "Close");
		_close.setLED(UserKeyAlignment.BottomRight, UserKeyLED.Red, UserKeyLEDSize.Normal);
		
		// Set WP PLUS key text, and add listener
		nNewWorkpiece = _gripperIO.getActWorkpiece();
		_wpPlus = _keyBar.addUserKey(2, _wpPlusListener, true);
		_wpPlus.setText(UserKeyAlignment.Middle, "+ " + nNewWorkpiece);
		
		// Set WP PLUS key text, and add listener
		_wpMinus = _keyBar.addUserKey(3, _wpMinusListener, true);
		_wpMinus.setText(UserKeyAlignment.Middle, "-");
		
		// Publish
		_keyBar.publish();
		
	}

	/** runCyclic @Override **/
	@Override public void runCyclic() {
		
		// Colour bits
		boolean _bRed   = false; // R
		boolean _bGreen = false; // G
		boolean _bBlue  = false; // B
		
		_logger.info("here1");
		// Gripper state for MF LED
		if((_gripper.isClosed())||(_gripper.isOpen())){_bBlue = true;}
		else{ _bRed = true; _bGreen = true; } // Moving
		
		_logger.info("here2");
        // Set media flange colours		
		_mfIO.setLEDRed(_bRed);
		_mfIO.setLEDGreen(_bGreen);
		_mfIO.setLEDBlue(_bBlue);
		
		_logger.info("here2");
		// Set process data
		_pdOpen.setValue(_gripper.isOpen());
		_pdClosed.setValue(_gripper.isClosed());
		_pdGrasped.setValue(_gripper.isGrasped());
		_pdWorkpiece.setValue(_gripper.getActWorkpiece());
		_logger.info("here3");
	}
	
	/** setLEDState - Update user key LED state **/
	private void setLEDState(){
        // User key colours
		if(_gripper.isOpen()){ 
			_open.setLED(UserKeyAlignment.BottomRight, UserKeyLED.Red, UserKeyLEDSize.Normal);
			_close.setLED(UserKeyAlignment.BottomRight, UserKeyLED.Green, UserKeyLEDSize.Normal);
		}
		else if(_gripper.isClosed()){ 
			_open.setLED(UserKeyAlignment.BottomRight, UserKeyLED.Green, UserKeyLEDSize.Normal);
			_close.setLED(UserKeyAlignment.BottomRight, UserKeyLED.Red, UserKeyLEDSize.Normal);
		}
		else{ 
			_open.setLED(UserKeyAlignment.BottomRight, UserKeyLED.Yellow, UserKeyLEDSize.Normal);
			_close.setLED(UserKeyAlignment.BottomRight, UserKeyLED.Yellow, UserKeyLEDSize.Normal);
		}
	}
}