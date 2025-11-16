package org.firstinspires.ftc.teamcode.Purple.Utils;

import java.lang.reflect.Field;

public class ReflectUtil
{
	/**
	 * Gets the value of a field by name from an object using reflection.
	 *
	 * @param obj       The object to get the field value from.
	 * @param fieldName The name of the field.
	 * @return The value of the field, or null if not found or inaccessible.
	 */
	public static Object getFieldValue (Object obj, String fieldName)
	{
		try
		{
			Field field = obj.getClass().getDeclaredField(fieldName);
			field.setAccessible(true);
			return field.get(obj);
		} catch (Exception e)
		{
			return null;
		}
	}
}
