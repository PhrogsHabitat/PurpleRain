package org.firstinspires.ftc.teamcode.Purple.Pathing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * PurpleChain is an ordered sequence of PurplePath objects.
 * It optionally contains a chain-level onComplete that is called
 * when the entire chain finishes.
 */
public class PurpleChain
{
	private final List<PurplePath> paths;
	private Runnable onComplete;

	/**
	 * Construct a chain from a varargs list of PurplePath.
	 *
	 * @param paths paths in execution order
	 */
	public PurpleChain (PurplePath... paths)
	{

		if (paths == null || paths.length == 0)
		{
			this.paths = new ArrayList<>();
		} else
		{
			this.paths = new ArrayList<>(Arrays.asList(paths));
		}
	}

	/** Returns an unmodifiable list of paths. */
	public List<PurplePath> getPaths ()
	{

		return Collections.unmodifiableList(paths);
	}

	/** Add a path to the end of the chain. */
	public PurpleChain add (PurplePath path)
	{

		this.paths.add(path);
		return this;
	}

	/**
	 * Set a chain-level onComplete callback that is invoked when all paths finish.
	 *
	 * @param onComplete runnable to run when whole chain finishes
	 * @return this (for chaining)
	 */
	public PurpleChain onComplete (Runnable onComplete)
	{

		this.onComplete = onComplete;
		return this;
	}

	/** Internal: run chain-level onComplete if present. */
	public void runOnComplete ()
	{

		if (onComplete != null)
		{
			try
			{
				onComplete.run();
			} catch (Exception e)
			{
				e.printStackTrace();
			}
		}
	}
}
