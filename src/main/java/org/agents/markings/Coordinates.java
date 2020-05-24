package org.agents.markings;

public enum Coordinates {
    TIME,//includes waiting time in a cell location
    ROW,// coordinate Y
    COLUMN;//corrdinate X

    //TO DO decapsulation of a new data strucure like a primitive array
    //TO DO bits for every field
    public static int[] createCoordinates(){
        return new int[Coordinates.getLenght()];
    }

    public static int[] createCoordinates(int time_step, int row_index, int col_index) {
         return new int[]{time_step, row_index, col_index };
    }

    public static int getTime(int[] pos){
        return getTime(0, pos);
    }

    public static int getTime(int index_pos, int [] pos){
        assert pos[index_pos * Coordinates.getLenght() + 2] >= 0;
        return pos[Coordinates.getLenght() * index_pos + Coordinates.TIME.ordinal()];
    }

    public static int getRow(int[] pos){
        return getRow(0, pos);
    }

    public static int getRow(int index_pos, int [] pos){
        return pos[Coordinates.values().length * index_pos + Coordinates.ROW.ordinal()];
    }

    public static int getCol(int[] pos){
        return getCol(0, pos);
    }

    public static int getCol(int index_pos, int [] pos){
        assert pos[index_pos * Coordinates.getLenght() + 2] >= 0;
        return pos[Coordinates.values().length * index_pos + Coordinates.COLUMN.ordinal()];
    }

    public static int getLenght(){
        return Coordinates.values().length;
    }


    public static void setTime(int[] pos, int value){
        setTime(0, pos, value);
    }

    public static void setTime(int index_pos, int[] pos, int value){
        assert pos[index_pos * Coordinates.getLenght() + 2] >= 0;
        pos[Coordinates.values().length * index_pos + Coordinates.TIME.ordinal()] = value;
    }

    public static void setRow(int[] pos, int value){
        setRow(0, pos, value);
    }

    public static void setRow(int index_pos, int[] pos, int value){
        assert pos[index_pos * Coordinates.getLenght() +2] >= -1;
        pos[Coordinates.values().length * index_pos + Coordinates.ROW.ordinal()] = value;
    }

    public static void setCol(int[] pos, int value){
        setCol(0, pos, value);
    }

    public static void setCol(int index_pos, int[] pos, int value){
        assert pos[index_pos * Coordinates.getLenght()] >= -1;
        pos[Coordinates.values().length * index_pos + Coordinates.COLUMN.ordinal()] = value;
    }

    public static int[] getCoordinatesAt(int index, int[] pos_coordinates) {
        int [] coord_pos = new int[0];
        for (int coordinate = 0; coordinate < pos_coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
                if (index == coordinate){
                    coord_pos = new int[Coordinates.getLenght()];
                    Coordinates.setTime(coord_pos, Coordinates.getTime(coordinate, pos_coordinates));
                    Coordinates.setRow(coord_pos, Coordinates.getRow(coordinate, pos_coordinates));
                    Coordinates.setCol(coord_pos, Coordinates.getCol(coordinate, pos_coordinates));
                }
        }
        return coord_pos;
    }

    public static void setCoordinateAtIndex(int index, int[] coordinates, int[] coordinate) {
        if(!isValid(coordinates) || !isValid(coordinate)){
            throw new IndexOutOfBoundsException("coordinate not valid");
        }
        for (int i = 0; i < coordinates.length; i = i + Coordinates.getLenght()) {
            if (i == index){
                Coordinates.setTime(index, coordinates, Coordinates.getTime(coordinate));
                Coordinates.setRow(index, coordinates, Coordinates.getRow(coordinate));
                Coordinates.setCol(index, coordinates, Coordinates.getCol(coordinate));
            }
        }
    }

    public static boolean isValid(int[] pos_coordinates){
        //if it has a lenght multiple of of the coordinates lenght is valid
        return (pos_coordinates.length % Coordinates.getLenght() == 0) ;
    }
}
//https://docs.oracle.com/javase/1.5.0/docs/guide/language/enums.html
//https://stackoverflow.com/questions/9254637/java-enum-vs-int
//https://docs.oracle.com/javase/10/docs/api/java/util/EnumSet.html
//https://jaxenter.com/top-10-easy-performance-optimisations-java-114952.html
/*
public enum ExampleEnum {
    value1(1),
    value2(2),
    valueUndefined(Integer.MAX_VALUE);

    private final int enumValue;
    private static Map enumMap;
    ExampleEnum(int value){
       enumValue = value;
    }
    static {
       enumMap = new HashMap<Integer, ExampleEnum>();
       for (ExampleEnum exampleEnum: ExampleEnum.values()) {
           enumMap.put(exampleEnum.value, exampleEnum);
        }
    }
    public static ExampleEnum getExampleEnum(int value) {
        return enumMap.contains(value) ? enumMap.get(value) : valueUndefined;
    }
}

* */