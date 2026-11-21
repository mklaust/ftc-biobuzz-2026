package org.firstinspires.ftc.team19938;

public class SpindexerIndexer {
    private final boolean[] slots = new boolean[5];
    public static final int SPINDEXER_CAPACITY = 4; // Capped strictly at 4 balls
    private int currentInputSlotIndex = 0;

    public int getBallCount() {
        int count = 0;
        for (boolean occupied : slots) {
            if (occupied) count++;
        }
        return count;
    }

    public boolean isFull() {
        return getBallCount() >= SPINDEXER_CAPACITY;
    }

    public void indexBallClockwise() {
        slots[currentInputSlotIndex] = true;
        // Advance array index 1 position clockwise (modulo 5)
        currentInputSlotIndex = (currentInputSlotIndex + 1) % 5;
    }

    public void resetTracker() {
        for (int i = 0; i < slots.length; i++) {
            slots[i] = false;
        }
        currentInputSlotIndex = 0; // Re-align empty slot to primary input
    }

    public boolean[] getSlotsState() {
        return slots.clone();
    }
}
