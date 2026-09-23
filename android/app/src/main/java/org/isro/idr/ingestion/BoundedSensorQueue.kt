package org.isro.idr.ingestion

import java.util.ArrayDeque

/**
 * Thread-safe bounded ring buffer for sensor event queuing.
 * Enforces hard memory boundaries and records overflow drop metrics.
 */
class BoundedSensorQueue<T>(
    val capacity: Int = 500
) {
    private val lock = Any()
    private val deque = ArrayDeque<T>(capacity)

    var totalPushed: Long = 0
        private set
    var totalPopped: Long = 0
        private set
    var totalDroppedOnOverflow: Long = 0
        private set

    val size: Int
        get() = synchronized(lock) { deque.size }

    val isFull: Boolean
        get() = synchronized(lock) { deque.size >= capacity }

    val isEmpty: Boolean
        get() = synchronized(lock) { deque.isEmpty() }

    /**
     * Pushes a new item into the queue.
     * If the queue is at capacity, drops the oldest item and increments drop counter.
     */
    fun push(item: T): Boolean = synchronized(lock) {
        if (deque.size >= capacity) {
            deque.pollFirst() // Discard oldest
            totalDroppedOnOverflow++
        }
        deque.addLast(item)
        totalPushed++
        true
    }

    /**
     * Polls the next oldest item from the queue, or returns null if empty.
     */
    fun poll(): T? = synchronized(lock) {
        val item = deque.pollFirst()
        if (item != null) totalPopped++
        item
    }

    /**
     * Drains all currently queued items into the destination list in FIFO order.
     */
    fun drainTo(destination: MutableList<T>): Int = synchronized(lock) {
        val count = deque.size
        while (!deque.isEmpty()) {
            destination.add(deque.pollFirst()!!)
            totalPopped++
        }
        count
    }

    fun clear() = synchronized(lock) {
        deque.clear()
    }
}
