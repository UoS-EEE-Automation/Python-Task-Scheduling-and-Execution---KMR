package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>ScannerSignals</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * Contains necessary information from the laser scanners of the AGV
 */
@Singleton
public class ScannerSignalsIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'ScannerSignals'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'ScannerSignals'
	 */
	@Inject
	public ScannerSignalsIOGroup(Controller controller)
	{
		super(controller, "ScannerSignals");

		addInput("WarningField_B1", IOTypes.BOOLEAN, 1);
		addInput("WarningField_B4", IOTypes.BOOLEAN, 1);
		addInput("WarningFieldComplete", IOTypes.BOOLEAN, 1);
		addInput("ProtectionField_B1", IOTypes.BOOLEAN, 1);
		addInput("ProtectionField_B4", IOTypes.BOOLEAN, 1);
		addInput("ProtectionFieldComplete", IOTypes.BOOLEAN, 1);
		addInput("ActiveScanField_B1", IOTypes.INTEGER, 16);
		addInput("ProtectionFieldMuted", IOTypes.BOOLEAN, 1);
		addInput("LastActiveScanField_B1", IOTypes.INTEGER, 16);
		addInput("ActiveScanField_B4", IOTypes.INTEGER, 16);
		addInput("LastActiveScanField_B4", IOTypes.INTEGER, 16);
		addInput("DirtDetection_B1", IOTypes.BOOLEAN, 1);
		addInput("DirtDetection_B4", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RequestCustomerField_07", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RequestCustomerField_08", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RequestCustomerField_09", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RequestCustomerField_10", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RequestCustomerField_11", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RequestCustomerField_12", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RequestCustomerField_13", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RequestCustomerField_14", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RequestCustomerField_15", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital input '<i>WarningField_B1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * 1 = Warning Field from Scanner B1 is free.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'WarningField_B1'
	 */
	public boolean getWarningField_B1()
	{
		return getBooleanIOValue("WarningField_B1", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>WarningField_B4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * 1 = Warning Field from Scanner B4 is free.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'WarningField_B4'
	 */
	public boolean getWarningField_B4()
	{
		return getBooleanIOValue("WarningField_B4", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>WarningFieldComplete</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * 1 = Warning Field from every Scanner is free.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'WarningFieldComplete'
	 */
	public boolean getWarningFieldComplete()
	{
		return getBooleanIOValue("WarningFieldComplete", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>ProtectionField_B1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * 1 = Protection Field from Scanner B1 is free
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'ProtectionField_B1'
	 */
	public boolean getProtectionField_B1()
	{
		return getBooleanIOValue("ProtectionField_B1", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>ProtectionField_B4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * 1 = Protection Field from Scanner B4 is free
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'ProtectionField_B4'
	 */
	public boolean getProtectionField_B4()
	{
		return getBooleanIOValue("ProtectionField_B4", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>ProtectionFieldComplete</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * 1 = Protection Field from eyery Scanner is free 
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'ProtectionFieldComplete'
	 */
	public boolean getProtectionFieldComplete()
	{
		return getBooleanIOValue("ProtectionFieldComplete", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>ActiveScanField_B1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Information witch Scanfield number is already active from 0 to 15 (16 Cases).
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-32768; 32767]
	 *
	 * @return current value of the digital input 'ActiveScanField_B1'
	 */
	public java.lang.Integer getActiveScanField_B1()
	{
		return getNumberIOValue("ActiveScanField_B1", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>ProtectionFieldMuted</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Information if the protection fields are active or not. State is "true" means they are inactive.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'ProtectionFieldMuted'
	 */
	public boolean getProtectionFieldMuted()
	{
		return getBooleanIOValue("ProtectionFieldMuted", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>LastActiveScanField_B1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Number of the last active scanfield before protection field error. 
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-32768; 32767]
	 *
	 * @return current value of the digital input 'LastActiveScanField_B1'
	 */
	public java.lang.Integer getLastActiveScanField_B1()
	{
		return getNumberIOValue("LastActiveScanField_B1", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>ActiveScanField_B4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Information witch Scanfield number is already active from 0 to 15 (16 Cases).
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-32768; 32767]
	 *
	 * @return current value of the digital input 'ActiveScanField_B4'
	 */
	public java.lang.Integer getActiveScanField_B4()
	{
		return getNumberIOValue("ActiveScanField_B4", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>LastActiveScanField_B4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Number of the last active scanfield before protection field error. 
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-32768; 32767]
	 *
	 * @return current value of the digital input 'LastActiveScanField_B4'
	 */
	public java.lang.Integer getLastActiveScanField_B4()
	{
		return getNumberIOValue("LastActiveScanField_B4", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>DirtDetection_B1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * true = Glass of B1 is dirty
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'DirtDetection_B1'
	 */
	public boolean getDirtDetection_B1()
	{
		return getBooleanIOValue("DirtDetection_B1", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>DirtDetection_B4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * true = Glass of B4 is dirty
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'DirtDetection_B4'
	 */
	public boolean getDirtDetection_B4()
	{
		return getBooleanIOValue("DirtDetection_B4", false);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RequestCustomerField_07</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RequestCustomerField_07'
	 */
	public boolean getRequestCustomerField_07()
	{
		return getBooleanIOValue("RequestCustomerField_07", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RequestCustomerField_07</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RequestCustomerField_07'
	 */
	public void setRequestCustomerField_07(java.lang.Boolean value)
	{
		setDigitalOutput("RequestCustomerField_07", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RequestCustomerField_08</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RequestCustomerField_08'
	 */
	public boolean getRequestCustomerField_08()
	{
		return getBooleanIOValue("RequestCustomerField_08", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RequestCustomerField_08</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RequestCustomerField_08'
	 */
	public void setRequestCustomerField_08(java.lang.Boolean value)
	{
		setDigitalOutput("RequestCustomerField_08", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RequestCustomerField_09</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RequestCustomerField_09'
	 */
	public boolean getRequestCustomerField_09()
	{
		return getBooleanIOValue("RequestCustomerField_09", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RequestCustomerField_09</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RequestCustomerField_09'
	 */
	public void setRequestCustomerField_09(java.lang.Boolean value)
	{
		setDigitalOutput("RequestCustomerField_09", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RequestCustomerField_10</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RequestCustomerField_10'
	 */
	public boolean getRequestCustomerField_10()
	{
		return getBooleanIOValue("RequestCustomerField_10", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RequestCustomerField_10</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RequestCustomerField_10'
	 */
	public void setRequestCustomerField_10(java.lang.Boolean value)
	{
		setDigitalOutput("RequestCustomerField_10", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RequestCustomerField_11</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RequestCustomerField_11'
	 */
	public boolean getRequestCustomerField_11()
	{
		return getBooleanIOValue("RequestCustomerField_11", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RequestCustomerField_11</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RequestCustomerField_11'
	 */
	public void setRequestCustomerField_11(java.lang.Boolean value)
	{
		setDigitalOutput("RequestCustomerField_11", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RequestCustomerField_12</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RequestCustomerField_12'
	 */
	public boolean getRequestCustomerField_12()
	{
		return getBooleanIOValue("RequestCustomerField_12", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RequestCustomerField_12</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RequestCustomerField_12'
	 */
	public void setRequestCustomerField_12(java.lang.Boolean value)
	{
		setDigitalOutput("RequestCustomerField_12", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RequestCustomerField_13</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RequestCustomerField_13'
	 */
	public boolean getRequestCustomerField_13()
	{
		return getBooleanIOValue("RequestCustomerField_13", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RequestCustomerField_13</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RequestCustomerField_13'
	 */
	public void setRequestCustomerField_13(java.lang.Boolean value)
	{
		setDigitalOutput("RequestCustomerField_13", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RequestCustomerField_14</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RequestCustomerField_14'
	 */
	public boolean getRequestCustomerField_14()
	{
		return getBooleanIOValue("RequestCustomerField_14", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RequestCustomerField_14</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RequestCustomerField_14'
	 */
	public void setRequestCustomerField_14(java.lang.Boolean value)
	{
		setDigitalOutput("RequestCustomerField_14", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RequestCustomerField_15</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RequestCustomerField_15'
	 */
	public boolean getRequestCustomerField_15()
	{
		return getBooleanIOValue("RequestCustomerField_15", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RequestCustomerField_15</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RequestCustomerField_15'
	 */
	public void setRequestCustomerField_15(java.lang.Boolean value)
	{
		setDigitalOutput("RequestCustomerField_15", value);
	}

}
