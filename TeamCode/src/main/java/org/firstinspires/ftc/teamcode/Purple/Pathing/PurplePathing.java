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
	private boolean waitingBetweenPaths = false;
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
	 * When the path finishes (or times out), its onComplete will run.
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
	 * Start a PurpleChain. The chain's paths are executed in order. After each path finishes,
	 * the path's onComplete is run and then the manager waits the configured waitTime before
	 * starting the next path. When the chain finishes, chainOnComplete runs.
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
			boolean followerDone = !follower.isBusy();
			boolean timedOut = pathTimer.getElapsedTimeSeconds() >= currentPath.getDurationSec();

			if (followerDone || timedOut)
			{
				// path finished
				pathActive = false;
				currentPath.runOnComplete();

				// If we are running a chain, start waiting between paths (if any remain)
				if (chainActive && currentChain != null)
				{
					// start wait (could be zero)
					waitingBetweenPaths = true;
					waitTimer.resetTimer();
				} else
				{
					// single path finished, clear currentPath (chain not running)
					currentPath = null;

					// if there was a stand-alone followPath call and it had an onComplete, we already called it.
				}
			}
		} else if (waitingBetweenPaths && currentChain != null)
		{
			double waited = waitTimer.getElapsedTimeSeconds();
			double waitFor = currentPath == null ? 0.0 : currentPath.getWaitTimeSec(); // last path's waitTime

			if (waited >= waitFor)
			{
				// proceed to next path in chain
				waitingBetweenPaths = false;
				currentChainIndex++;

				if (currentChainIndex >= currentChain.getPaths().size())
				{
					// chain finished
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
					// run chain-level onComplete stored in chain itself as well
					finishedChain.runOnComplete();
				} else
				{
					// start next
					PurplePath next = currentChain.getPaths().get(currentChainIndex);
					startPathInternal(next, currentHoldEnd);
				}
			}
		}
	}

	private void startPathInternal (PurplePath path, boolean holdEnd)
	{

		this.currentPath = path;
		this.pathActive = true;
		this.waitingBetweenPaths = false;
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
