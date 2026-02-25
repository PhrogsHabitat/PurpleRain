package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

import android.content.Context;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Properties;

/**
 * Simple persistent key-value storage backed by a file on the Robot Controller.
 */
public class Persist
{
	private static final String FILE_NAME = "PurplePersist.properties";

	private final File file;

	/**
	 * Creates persistent storage using the current Robot Controller app context.
	 */
	public Persist (HardwareMap hardwareMap)
	{
		this(hardwareMap.appContext);
	}

	/**
	 * Creates persistent storage using the provided Android context.
	 */
	public Persist (Context context)
	{
		this.file = new File(context.getFilesDir(), FILE_NAME);
	}

	/**
	 * Saves a key/value pair. If value is null, the key is removed.
	 */
	public synchronized void set (String key, String value)
	{
		validateKey(key);

		Properties properties = loadProperties();
		if (value == null)
		{
			properties.remove(key);
		}
		else
		{
			properties.setProperty(key, value);
		}

		saveProperties(properties);
	}

	/**
	 * Gets a value by key, or null when the key does not exist.
	 */
	public synchronized String get (String key)
	{
		validateKey(key);
		return loadProperties().getProperty(key);
	}

	/**
	 * Gets a value by key, or the provided fallback when missing.
	 */
	public synchronized String get (String key, String fallback)
	{
		validateKey(key);
		return loadProperties().getProperty(key, fallback);
	}

	private Properties loadProperties ()
	{
		Properties properties = new Properties();

		if (!file.exists())
		{
			return properties;
		}

		try (FileInputStream input = new FileInputStream(file))
		{
			properties.load(input);
		}
		catch (IOException ignored)
		{
			// Return an empty map on read errors to keep usage simple in OpModes.
		}

		return properties;
	}

	private void saveProperties (Properties properties)
	{
		try (FileOutputStream output = new FileOutputStream(file))
		{
			properties.store(output, "Purple Persist");
		}
		catch (IOException e)
		{
			throw new RuntimeException("Failed to write persist storage file", e);
		}
	}

	private void validateKey (String key)
	{
		if (key == null || key.trim().isEmpty())
		{
			throw new IllegalArgumentException("Key cannot be null or blank");
		}
	}
}
