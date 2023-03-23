package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>ZimmerHRC</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
@Singleton
public class ZimmerHRCIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'ZimmerHRC'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'ZimmerHRC'
	 */
	@Inject
	public ZimmerHRCIOGroup(Controller controller)
	{
		super(controller, "ZimmerHRC");

		addInput("IsClosed", IOTypes.BOOLEAN, 1);
		addInput("IsGrasped", IOTypes.BOOLEAN, 1);
		addInput("IsReleased", IOTypes.BOOLEAN, 1);
		addInput("MotorON", IOTypes.BOOLEAN, 1);
		addInput("HomingOK", IOTypes.BOOLEAN, 1);
		addInput("Error", IOTypes.BOOLEAN, 1);
		addInput("ActWorkpiece", IOTypes.UNSIGNED_INTEGER, 4);
		addDigitalOutput("CmdGrasp", IOTypes.BOOLEAN, 1);
		addDigitalOutput("CmdHoming", IOTypes.BOOLEAN, 1);
		addDigitalOutput("CmdMotor", IOTypes.BOOLEAN, 1);
		addDigitalOutput("CmdRelease", IOTypes.BOOLEAN, 1);
		addDigitalOutput("CmdReset", IOTypes.BOOLEAN, 1);
		addDigitalOutput("CmdWorkpiece", IOTypes.UNSIGNED_INTEGER, 4);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IsClosed</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IsClosed'
	 */
	public boolean getIsClosed()
	{
		return getBooleanIOValue("IsClosed", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IsGrasped</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IsGrasped'
	 */
	public boolean getIsGrasped()
	{
		return getBooleanIOValue("IsGrasped", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IsReleased</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IsReleased'
	 */
	public boolean getIsReleased()
	{
		return getBooleanIOValue("IsReleased", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>MotorON</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'MotorON'
	 */
	public boolean getMotorON()
	{
		return getBooleanIOValue("MotorON", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>HomingOK</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'HomingOK'
	 */
	public boolean getHomingOK()
	{
		return getBooleanIOValue("HomingOK", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Error</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Error'
	 */
	public boolean getError()
	{
		return getBooleanIOValue("Error", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>ActWorkpiece</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 15]
	 *
	 * @return current value of the digital input 'ActWorkpiece'
	 */
	public java.lang.Integer getActWorkpiece()
	{
		return getNumberIOValue("ActWorkpiece", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital output '<i>CmdGrasp</i>'</b>.<br>
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
	 * @return current value of the digital output 'CmdGrasp'
	 */
	public boolean getCmdGrasp()
	{
		return getBooleanIOValue("CmdGrasp", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>CmdGrasp</i>'</b>.<br>
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
	 *            the value, which has to be written to the digital output 'CmdGrasp'
	 */
	public void setCmdGrasp(java.lang.Boolean value)
	{
		setDigitalOutput("CmdGrasp", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>CmdHoming</i>'</b>.<br>
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
	 * @return current value of the digital output 'CmdHoming'
	 */
	public boolean getCmdHoming()
	{
		return getBooleanIOValue("CmdHoming", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>CmdHoming</i>'</b>.<br>
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
	 *            the value, which has to be written to the digital output 'CmdHoming'
	 */
	public void setCmdHoming(java.lang.Boolean value)
	{
		setDigitalOutput("CmdHoming", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>CmdMotor</i>'</b>.<br>
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
	 * @return current value of the digital output 'CmdMotor'
	 */
	public boolean getCmdMotor()
	{
		return getBooleanIOValue("CmdMotor", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>CmdMotor</i>'</b>.<br>
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
	 *            the value, which has to be written to the digital output 'CmdMotor'
	 */
	public void setCmdMotor(java.lang.Boolean value)
	{
		setDigitalOutput("CmdMotor", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>CmdRelease</i>'</b>.<br>
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
	 * @return current value of the digital output 'CmdRelease'
	 */
	public boolean getCmdRelease()
	{
		return getBooleanIOValue("CmdRelease", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>CmdRelease</i>'</b>.<br>
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
	 *            the value, which has to be written to the digital output 'CmdRelease'
	 */
	public void setCmdRelease(java.lang.Boolean value)
	{
		setDigitalOutput("CmdRelease", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>CmdReset</i>'</b>.<br>
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
	 * @return current value of the digital output 'CmdReset'
	 */
	public boolean getCmdReset()
	{
		return getBooleanIOValue("CmdReset", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>CmdReset</i>'</b>.<br>
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
	 *            the value, which has to be written to the digital output 'CmdReset'
	 */
	public void setCmdReset(java.lang.Boolean value)
	{
		setDigitalOutput("CmdReset", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>CmdWorkpiece</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 15]
	 *
	 * @return current value of the digital output 'CmdWorkpiece'
	 */
	public java.lang.Integer getCmdWorkpiece()
	{
		return getNumberIOValue("CmdWorkpiece", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>CmdWorkpiece</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 15]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'CmdWorkpiece'
	 */
	public void setCmdWorkpiece(java.lang.Integer value)
	{
		setDigitalOutput("CmdWorkpiece", value);
	}

}
