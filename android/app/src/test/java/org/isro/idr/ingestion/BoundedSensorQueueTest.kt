package org.isro.idr.ingestion

import org.junit.Assert.*
import org.junit.Test

class BoundedSensorQueueTest {

    @Test
    fun testFifoOrderingAndCapacityEnforcement() {
        val queue = BoundedSensorQueue<Int>(capacity = 5)

        for (i in 1..5) {
            queue.push(i)
        }
        assertEquals(5, queue.size)
        assertTrue(queue.isFull)
        assertEquals(0L, queue.totalDroppedOnOverflow)

        // Push 6th element -> drops oldest (1)
        queue.push(6)
        assertEquals(5, queue.size)
        assertEquals(1L, queue.totalDroppedOnOverflow)

        // Drain to list
        val drained = mutableListOf<Int>()
        val count = queue.drainTo(drained)

        assertEquals(5, count)
        assertEquals(listOf(2, 3, 4, 5, 6), drained)
        assertTrue(queue.isEmpty)
    }

    @Test
    fun testPollFromEmptyQueue() {
        val queue = BoundedSensorQueue<String>(capacity = 10)
        assertNull(queue.poll())
        assertTrue(queue.isEmpty)
    }
}
