package org.firstinspires.ftc.teamcode.Purple.Memory;

import java.util.HashMap;
import java.util.Map;

/**
 * A simple key-value memory store for runtime data.
 */
public class PurpleMemory
{
    private final Map<String, Object> memory = new HashMap<>();

    /**
     * Stores a value in memory under the given key.
     *
     * @param key   The key to store the value under.
     * @param value The value to store.
     */
    public void put(String key, Object value)
    {
        memory.put(key, value);
    }

    /**
     * Retrieves a value from memory by key.
     * @param key The key to look up.
     * @return The value associated with the key, or null if not found.
     */
    public Object get(String key)
    {
        return memory.get(key);
    }

    /**
     * Checks if the memory contains a value for the given key.
     * @param key The key to check.
     * @return True if the key exists, false otherwise.
     */
    public boolean containsKey(String key)
    {
        return memory.containsKey(key);
    }

    /**
     * Clears all values from memory.
     */
    public void clear()
    {
        memory.clear();
    }
}
