package frc.robot;

import java.util.Iterator;
import java.util.NoSuchElementException;

public class RingBuffer<T> {
    private int size;
    private Object[] buffer;
    private int index;

    public RingBuffer(int size) {
        this.size = size;
        index = 0;
        buffer = new Object[size];
    }

    public void push(T value) {
        buffer[index] = value;
        index++;
        if (index == size) {
            index = 0;
        }
    }

    public class RingBufferIterator implements Iterator<T> {
        private int currentIndex;
        private RingBuffer<T> buffer;
        private boolean hasLooped;

        private RingBufferIterator(RingBuffer<T> buffer) {
            this.buffer = buffer;
            currentIndex = buffer.index - 1;
            if (currentIndex < 0) {
                currentIndex = buffer.size;
                hasLooped = true;
            }
            hasLooped = false;
        }

        @Override
        public boolean hasNext() {
            if (hasLooped) {
                return this.currentIndex >= buffer.index;
            } else {
                return true;
            }
        }

        @Override
        public T next() {
            int index = currentIndex;
            currentIndex--;
            if (currentIndex < 0) {
                hasLooped = true;
                currentIndex = buffer.size;
            }

            if (hasLooped && currentIndex == buffer.index - 1) {
                throw new NoSuchElementException();
            }

            return (T)buffer.buffer[index];
        }

    }
}
