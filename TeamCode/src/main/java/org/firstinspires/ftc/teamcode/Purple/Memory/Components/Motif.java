package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;

/**
 * Tracks the current motif from AprilTag IDs.
 * 21 = GPP, 22 = PGP, 23 = PPG
 */
public class Motif
{
	public enum Type
	{
		GPP,
		PGP,
		PPG
	}

	public static final Type GPP = Type.GPP;
	public static final Type PGP = Type.PGP;
	public static final Type PPG = Type.PPG;

	private Type curMotif = Type.GPP;

	/**
	 * Updates the motif from the current AprilTag ID.
	 */
	public void update ()
	{
		int tagId = LimeUtil.getPrimaryFiducialId();
		switch (tagId)
		{
			case 21:
				curMotif = Type.GPP;
				break;
			case 22:
				curMotif = Type.PGP;
				break;
			case 23:
				curMotif = Type.PPG;
				break;
			default:
				// Keep last known motif when no valid motif tag is visible.
				break;
		}
	}

	/**
	 * Returns the current motif.
	 */
	public Type curMotif ()
	{
		return curMotif;
	}
}
