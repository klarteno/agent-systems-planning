package org.agents;

import java.io.BufferedReader;
import java.util.HashMap;
import java.util.Optional;
import java.util.Vector;
import java.util.regex.Pattern;


public final class SearchClient {
    BufferedReader serverMessages;

    enum ParsingState {
        NOTHING,
        COLORS,
        GOAL_MAP,
        END,
        INIT_MAP;
    };

    HashMap<String, Color> colors;
    int maxCol_map = 0;
    int row_map_no_ = 0;
    Vector<char[]> rows_init_map_marks;
    Vector<char[]> goal_map_marks;

    static final Pattern pattern_text_headers = Pattern.compile("^[#]?[a-z]+$");
    static final Pattern pattern_colors_headers = Pattern.compile("^[#]?color[a-z]*$");
    static final Pattern pattern_initial_map_headers = Pattern.compile("^[#]?init[a-z]*$");
    static final Pattern pattern_goal_map_headers = Pattern.compile("^[#]?goal[a-z]*$");
    static final Pattern pattern_end_headers = Pattern.compile("^[#]?end[a-z]*$");

    public SearchClient(BufferedReader serverMessages) throws Exception {
        this.serverMessages = serverMessages;
        //color for object parsed from the map
        this.colors 	= new HashMap<>();
        this.rows_init_map_marks = new Vector<char[]>(3);
        this.goal_map_marks = new Vector<char[]>(3);;
    }

    public void parse() throws Exception {

        ParsingState parsingState = ParsingState.NOTHING;

        for (String line; (line = serverMessages.readLine()) != null && !line.equals("");)
        {
           /* if (pattern_text_headers.matcher(line).matches()){
                //System.out.println("#found " + line.toString());
            }*/
            /*
            if (pattern_text_headers.matcher(line).group().indexOf("color")>0){
                System.out.println("#colors found " + line.toString());
            }*/
            if (pattern_colors_headers.matcher(line).matches()){
                parsingState = ParsingState.COLORS;
                continue;
            }

            if (pattern_initial_map_headers.matcher(line).matches()){
                parsingState = ParsingState.INIT_MAP;

                continue;
            }

            if (pattern_goal_map_headers.matcher(line).matches()){
                parsingState = ParsingState.GOAL_MAP;

                continue;
            }

            if (pattern_end_headers.matcher(line).matches()){
                parsingState = ParsingState.END;
            }
/*maybe will not be used
            if(parsingState==ParsingState.COLORS){
                    Vector<String> colors_parsed = new Vector<String>();

                HashMap<String, Vector<String>> colors 	= new HashMap<>();
                String[] strings_parsed = line.split("[:,]");

                Vector<String> same_color = new Vector<String>(strings_parsed.length - 1);
                for (int i = 1; i < strings_parsed.length; i++) {
                    same_color.add(strings_parsed[i]);
                    //System.out.println("#strs:  "+ i + strs[i]);
                }
                colors.put(strings_parsed[0],same_color);

                continue;
            }
 */
            if(parsingState == ParsingState.COLORS){

                String[] strings_parsed = line.split("[:,]");
                Color color = Color.geFromName(strings_parsed[0].trim().toLowerCase());

                for (int i = 1; i < strings_parsed.length; i++) {
                    colors.put(strings_parsed[i].trim(), color);
                }

                continue;
            }

            if(parsingState == ParsingState.INIT_MAP){
                row_map_no_++;

                maxCol_map = parse_map_data(maxCol_map, rows_init_map_marks, line);
                assert rows_init_map_marks.size()>5;

                continue;
            }

            //System.out.println("#"+line.toString());
            //System.err.println(line.toString());
            if(parsingState == ParsingState.GOAL_MAP){
                parse_map_data(maxCol_map, goal_map_marks, line);

                continue;
            }

            if(parsingState == ParsingState.END){
                System.out.println("# end map parsing"+line.toString());

                break;
            }
        }
     }


    public MapFixedObjects initObjects() {
      return initObjects(this.rows_init_map_marks, this.goal_map_marks, this.colors, this.maxCol_map);
    }

    public MapFixedObjects initObjects(Vector<char[]> rows_init_map_marks, Vector<char[]> goal_map_marks, HashMap<String, Color> colors, int maxCol_map){
        MapFixedObjects mapFixedObjects = new MapFixedObjects();

        MapFixedObjects.MAX_COL = maxCol_map;


        MapFixedObjects.MAX_ROW = rows_init_map_marks.size();
        MapFixedObjects.setWalls(new boolean[MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL]);

        var box_marks = new Vector<Box>();
        var agent_marks = new Vector<Agent>();

        int row = 0;
        for (char[] cs : rows_init_map_marks) {
            for (int j = 0; j < cs.length; j++) {
                if (cs[j] == '+') {// Wall
                    MapFixedObjects.getWalls()[row][j] = true;
                } else if ('A' <= goal_map_marks.get(row)[j] && goal_map_marks.get(row)[j] <= 'Z') { // Goal Box.
                    char character = goal_map_marks.get(row)[j];
                    MapFixedObjects.goals.put(character,new int[]{row, j});
                    //goals_marks.add(new Goal(row, j, cs[j]));
                } else if ('0' <= goal_map_marks.get(row)[j] && goal_map_marks.get(row)[j] <= '9') { // Goal Agent.
                    MapFixedObjects.goals.put(goal_map_marks.get(row)[j],new int[]{row, j});
                } else if ('A' <= cs[j] && cs[j] <= 'Z') { // Box.
                    int no = Color.getColorCoded(colors.remove(Character.toString(cs[j])));
                    Box box = new Box(cs[j], no);
                    box.setRowPosition(row);
                    box.setColumnPosition(j);
                    box_marks.add(box);
                } else if ('0' <= cs[j] && cs[j] <= '9') { // Agent.
                    Integer color_mark = Color.getColorCoded(colors.get(Character.toString(cs[j])));
                    Integer number_mark = Character.getNumericValue(cs[j]);
                    Agent agent = new Agent(number_mark, color_mark);
                    agent.setRowPosition(row);
                    agent.setColumnPosition(j);
                    agent_marks.add(agent);
                }
            }
            row++;
        }

        for (Character key: MapFixedObjects.goals.keySet()) {
            int[] coordinates = MapFixedObjects.goals.get(key);
            if ('A' <= key && key <= 'Z'){
                for (Box box : box_marks) {
                    if (box.getLetterMark() == Character.getNumericValue(key)) {
                        box.setGoalPosition(coordinates[0], coordinates[1]);
                    }
                }
            }
            else if ('0' <= key && key <= '9'){
                for (Agent agent : agent_marks) {
                    if (agent.getNumberMark() == Character.getNumericValue(key)) {
                        agent.setGoalPosition(coordinates[0], coordinates[1]);
                    }
                }
            }
            else {
                System.out.println("#Character key: MapFixedObjects.goals.keySet() error");
            }
        }
         MapFixedObjects.setMovables(agent_marks.toArray(new Agent[0]), box_marks.toArray(new Box[0]));

        return mapFixedObjects;
    }



    private int parse_map_data(int maxCol_map_no, Vector<char[]> rows_map_marks, String line) {
        int columns = line.length();
        rows_map_marks.add(new char[columns]);
        int row_map_no = rows_map_marks.size() - 1;

        for (int col = 0; col < columns; col++) {
            char chr = line.charAt(col);

            if (chr == '+') { // Wall.
                rows_map_marks.elementAt(row_map_no)[col] = chr;
            } else if ('0' <= chr && chr <= '9') { // Agent.
                rows_map_marks.elementAt(row_map_no)[col] = chr;
            } else if ('A' <= chr && chr <= 'Z') { // Box.
                rows_map_marks.elementAt(row_map_no)[col] = chr;
            } else if ('a' <= chr && chr <= 'z') { // Goal.
                rows_map_marks.elementAt(row_map_no)[col] = chr;
            } else if (chr == ' ') {
                // Free space.
            } else {
                System.out.println("#Error, read invalid level character: " + (int) chr);
                System.exit(1);
            }
        }

        if (columns > maxCol_map_no)
            maxCol_map_no = columns;

        return maxCol_map_no;
    }


}
