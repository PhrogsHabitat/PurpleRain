package org.firstinspires.ftc.teamcode.Purple.Pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;

/**
 * PurplePathing manages following PurplePath objects using a Pedro Follower.
 * <p>
 * Responsibilities:
 * - start single paths (followPath)
 * - start chains of paths (startChain)
 * - enforce a safety timeout per-path (durationSec)
 * - wait the configured waitTimeSec between paths
 * - **ensure onComplete callbacks are executed only after a path has
 *   finished and its associated waitTime has elapsed**
 * - run per-path onComplete and chain-level onComplete
 * <p>
 * Usage:
 * PurplePathing pm = new PurplePathing(follower);
 * pm.followPath(pathA, true); // single path
 * <p>
 * // or chain:
 * pm.startChain(chain, false, () -> { DebugUtil.logAdd("chain done"); });
 * <p>
 * IMPORTANT: call update() from your OpMode loop(); it does not spawn threads.
 */
public class PurplePathing
{

	private final Follower follower;

	// per-path timing
	private final Timer pathTimer = new Timer();
	private final Timer waitTimer = new Timer();

	// single path control
	private PurplePath currentPath = null;
	private boolean pathActive = false;
	// true when we've finished a path and are delaying until the
	// associated waitTime has expired; this flag is used for both chains
	// and standalone paths.
	private boolean waitingAfterPath = false;
	private boolean currentHoldEnd = true;

	// chain control
	private PurpleChain currentChain = null;
	private int currentChainIndex = 0;
	private Runnable chainOnComplete = null;
	private boolean chainActive = false;

	/**
	 * Construct the manager with a Pedro Follower instance.
	 *
	 * @param follower Pedro Follower (created using your Constants.createFollower)
	 */
	public PurplePathing (Follower follower)
	{

		this.follower = follower;
	}

	/**
	 * Start following a single PurplePath.
	 * The provided path's onComplete callback will be invoked **after the path
	 * completes (or times out) and its configured waitTime has elapsed**.  This
	 * makes the behaviour consistent with chains and avoids firing callbacks as
	 * soon as the follower begins moving.
	 *
	 * @param path    the PurplePath to follow
	 * @param holdEnd whether to hold the drivetrain at the end of the path (passed to follower.followPath)
	 */
	public void followPath (PurplePath path, boolean holdEnd)
	{
		// cancel any running chain
		currentChain = null;
		chainActive = false;
		chainOnComplete = null;

		startPathInternal(path, holdEnd);
	}

	/**
	 * Start a PurpleChain. The chain's paths are executed in order. After each path finishes
	 * the manager waits the configured waitTime before invoking that path's onComplete and
	 * then starting the next path. When the chain finishes, chainOnComplete runs.
	 *
	 * @param chain           the PurpleChain to run
	 * @param holdEnd         whether each path should be started with holdEnd (passed to follower.followPath)
	 * @param chainOnComplete optional runnable to run when the entire chain completes
	 */
	public void startChain (PurpleChain chain, boolean holdEnd, Runnable chainOnComplete)
	{

		if (chain == null || chain.getPaths().isEmpty()) return;

		this.currentChain = chain;
		this.currentChainIndex = 0;
		this.chainOnComplete = chainOnComplete;
		this.chainActive = true;
		this.currentHoldEnd = holdEnd;

		// start first
		startPathInternal(chain.getPaths().get(0), holdEnd);
	}

	/**
	 * Must be called inside the OpMode loop() on every iteration.
	 * This updates the follower and handles completion, timeouts, wait timers and chain advancement.
	 */
	public void update ()
	{
		// keep Pedro updated (important)
		follower.update();

		if (pathActive && currentPath != null)
		{
			// ensure that the expected duration has elapsed before considering the
			// path complete. the Pedro follower may report itself "not busy" very
			// quickly (sometimes immediately), which in the old version caused
			// onComplete to fire right away. requiring the timer to reach
			// durationSec guarantees callbacks happen after the intended period.
			// if the follower is still busy when the timer expires we treat it as a
			// timeout and move on anyway.
			double elapsed = pathTimer.getElapsedTimeSeconds();
			if (elapsed >= currentPath.getDurationSec())
			{
				pathActive = false;
				waitingAfterPath = true;
				waitTimer.resetTimer();
			}
		} else if (waitingAfterPath && currentPath != null)
		{
			double waited = waitTimer.getElapsedTimeSeconds();
			double waitFor = currentPath.getWaitTimeSec();

			if (waited >= waitFor)
			{
				// the post‑path delay has finished; now run the per‑path callback.
				currentPath.runOnComplete();

				// proceed based on whether we're in a chain or a standalone path
				if (chainActive && currentChain != null)
				{
					// move to the next path in the active chain
					waitingAfterPath = false;
					currentChainIndex++;

					if (currentChainIndex >= currentChain.getPaths().size())
					{
						// entire chain finished
						chainActive = false;
						PurpleChain finishedChain = currentChain;
						currentChain = null;
						currentPath = null;
						if (chainOnComplete != null)
						{
							try
							{
								chainOnComplete.run();
							} catch (Exception e)
							{
								e.printStackTrace();
							}
						}
						finishedChain.runOnComplete();
					} else
					{
						// start the next path in the chain
						PurplePath next = currentChain.getPaths().get(currentChainIndex);
						startPathInternal(next, currentHoldEnd);
					}
				} else
				{
					// single path finished; clear currentPath
					currentPath = null;
					waitingAfterPath = false;
				}
			}
		}
	}

	private void startPathInternal (PurplePath path, boolean holdEnd)
	{

		this.currentPath = path;
		this.pathActive = true;
		this.waitingAfterPath = false;
		this.pathTimer.resetTimer();

		// send to Pedro follower
		follower.followPath(path.getPath(), holdEnd);
	}

	/**
	 * Return a human-friendly description of the current path (or NONE).
	 * Includes chain index when a chain is active.
	 */
	public String curPath ()
	{

		if (currentChain != null && chainActive)
		{
			String name = (currentPath == null) ? "NONE" : currentPath.getName();
			return String.format("CHAIN[%d/%d] %s",
					Math.min(currentChainIndex + 1, currentChain.getPaths().size()),
					currentChain.getPaths().size(),
					name);
		} else if (currentPath != null)
		{
			return currentPath.getName();
		} else
		{
			return "NONE";
		}
	}
}
