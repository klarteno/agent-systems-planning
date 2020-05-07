package org.agents.action;

public class CellLocation {
/** Combines two integer coordinates into a single long.**/
 private static long point(int x, int y) {
    return (((long)x) << 32) | y;
 }

 /** Extracts the x coordinate from a point long.**/
 private static int getX(long point) {
    return (int)(point >> 32);
 }

 /** Extracts the y coordinate from a point long.**/
 private static int getY(long point) {
    return (int) point;
 }

}

//enum class with int is on the stack??
class Pair2 {
    private final int pair;

    public Pair2(short x, short y){
        pair =  (((int)x) << 16) | y;
    }

    public short getFirst() {
        return (short) (pair >> 16);
    }

    public short getSecond() {
        return (short) pair;
    }

    public int getPair() {
        return pair;
    }
}

class ConflictingPair extends Pair2 {

    public ConflictingPair(short x, short y) {
        super(x, y);
    }
}

class PairOf3 {
    private final int pair;

    public PairOf3(byte x, byte y, byte z){
        pair = (int)x << 16 |(int)y << 8 | z;
    }

    public byte getFirst() {
        return (byte)(pair >> 16);
    }

    public byte getSecond() {
        return (byte)(pair >> 8);
    }

    public byte getThird() {
        return (byte) pair;
    }

    public int getPair() {
        return pair;
    }
}


class CoordinateOf3 extends PairOf3 {

    public CoordinateOf3(byte x, byte y, byte z) {
        super(x, y, z);
    }
}