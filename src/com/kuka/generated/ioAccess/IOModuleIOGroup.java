package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>IOModule</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
@Singleton
public class IOModuleIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'IOModule'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'IOModule'
	 */
	@Inject
	public IOModuleIOGroup(Controller controller)
	{
		super(controller, "IOModule");

		addInput("Status_X4X5", IOTypes.BOOLEAN, 1);
		addInput("Status_X6", IOTypes.BOOLEAN, 1);
		addInput("Status_X7", IOTypes.BOOLEAN, 1);
		addInput("Error_X4", IOTypes.BOOLEAN, 1);
		addInput("Error_X5", IOTypes.BOOLEAN, 1);
		addInput("Error_X4X5", IOTypes.BOOLEAN, 1);
		addInput("Error_X6", IOTypes.BOOLEAN, 1);
		addInput("Error_X7", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Status_X4X5</i>'</b>.<br>
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
	 * @return current value of the digital input 'Status_X4X5'
	 */
	public boolean getStatus_X4X5()
	{
		return getBooleanIOValue("Status_X4X5", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Status_X6</i>'</b>.<br>
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
	 * @return current value of the digital input 'Status_X6'
	 */
	public boolean getStatus_X6()
	{
		return getBooleanIOValue("Status_X6", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Status_X7</i>'</b>.<br>
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
	 * @return current value of the digital input 'Status_X7'
	 */
	public boolean getStatus_X7()
	{
		return getBooleanIOValue("Status_X7", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Error_X4</i>'</b>.<br>
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
	 * @return current value of the digital input 'Error_X4'
	 */
	public boolean getError_X4()
	{
		return getBooleanIOValue("Error_X4", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Error_X5</i>'</b>.<br>
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
	 * @return current value of the digital input 'Error_X5'
	 */
	public boolean getError_X5()
	{
		return getBooleanIOValue("Error_X5", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Error_X4X5</i>'</b>.<br>
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
	 * @return current value of the digital input 'Error_X4X5'
	 */
	public boolean getError_X4X5()
	{
		return getBooleanIOValue("Error_X4X5", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Error_X6</i>'</b>.<br>
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
	 * @return current value of the digital input 'Error_X6'
	 */
	public boolean getError_X6()
	{
		return getBooleanIOValue("Error_X6", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Error_X7</i>'</b>.<br>
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
	 * @return current value of the digital input 'Error_X7'
	 */
	public boolean getError_X7()
	{
		return getBooleanIOValue("Error_X7", false);
	}

}
