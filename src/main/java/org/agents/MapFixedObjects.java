package org.agents;

import java.util.HashMap;

public class MapFixedObjects {
        public static int MAX_ROW = 0;
        public static int MAX_COL = 0;

        public static boolean[][] walls;
        //public static char[][] goals;
        //goals is not intended to use in the algorithms instead we store the goals in the box or agent object
        public static  HashMap<Character, int[]> goals = new HashMap<>();

        public static Box[] boxes;
        public static Agent[] agents;
        }
