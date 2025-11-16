package org.firstinspires.ftc.teamcode.Purple.Memory;

public class MemoryUtil
{
	/**
	 * Returns true if the array contains the given value.
	 *
	 * @param array The array to search.
	 * @param value The value to find.
	 * @param <T>   The type of the array elements.
	 * @return True if the value is found, false otherwise.
	 */
	public static <T> boolean contains (T[] array, T value)
	{
		for (T item : array)
		{
			if (item != null && item.equals(value))
			{
				return true;
			}
		}
		return false;
	}
}
