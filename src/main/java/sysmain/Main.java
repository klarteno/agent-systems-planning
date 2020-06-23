package sysmain;

 import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.SearchClient;
import org.agents.markings.Coordinates;
import org.agents.markings.SolvedStatus;
import org.agents.planning.GroupSearch;
import org.agents.planning.conflicts.ConflictAvoidanceTable;
import org.agents.planning.schedulling.*;
import org.agents.searchengine.PathProcessing;
import org.agents.searchengine.SearchTaskResult;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.DecimalFormat;
import java.util.*;

/**
 * This class is used to lunch project
 *
 * @author autor
 *  {@link ConflictAvoidanceTable} interface
 *   to handle updates of path
 */
public final class Main{

    enum SearchingMode {
        SINGLE,
        GROUP,
        MERGING,
        CANCELLING,
        GOAL_REACHED;
    };

    private static final DecimalFormat df = new DecimalFormat("0.0000");

    static int[][] total_group_copy;

    /**
     * This is the main method of our application.
     * @param args {@link String} Input arguments.
     */
    public static void main(String[] args)
    {
        try
        {
            long startTime = System.nanoTime();

            BufferedReader serverMessages = new BufferedReader(new InputStreamReader(System.in));
            // Use stderr to print to console
            System.out.println("#SearchClient initializing. I am sending this using the error output stream.");

            // Read level and create the initial state of the problem
            SearchClient client = new SearchClient(serverMessages);
            client.parse();

            MapFixedObjects mapFixedObjects = client.initObjects();

            mapFixedObjects.setUpTrackedMovables(MapFixedObjects.getAgents().toArray(new Agent[0]), MapFixedObjects.getBoxes().toArray(new Box[0]));

            ////////        entry--point        ///////////////////
            //ArrayList<String[]> pathOperatorDecompositionSearch = getPathOperatorDecompositionSearch();
            ArrayList<String[]> pathOperatorDecompositionSearch = getPathOperatorDecompositionScheduling();
            ////////        entry--point        ///////////////////

             ListIterator<String[]> path_iter = pathOperatorDecompositionSearch.listIterator(pathOperatorDecompositionSearch.size());
            ArrayDeque<ListIterator<String[]>> paths_iterations = new ArrayDeque<>();
            paths_iterations.add(path_iter);

            String[] no_of_slots = pathOperatorDecompositionSearch.get(0);
            outputPathsForAgents(serverMessages, total_group_copy, no_of_slots.length,  paths_iterations.pop());

            long endTime = System.nanoTime();
            String duration = df.format( (endTime - startTime) / 1000000000.0 );
            System.out.println("#Result time duration : " + duration    );
        }
        catch (Throwable e) {
            e.printStackTrace();
        }
    }

    private static void outputPathsForAgents(BufferedReader serverMessages, int[][] total_group_copy, int slots_length, ListIterator<String[]> path_iter) throws IOException {
        int[] index_agents = total_group_copy[0];
        int[] start_group = total_group_copy[1];

        String[] msg1 = new String[slots_length + 1];
        Arrays.fill(msg1, "NoOp");
        msg1[slots_length]=  System.lineSeparator();

        String[] msg_commnand_list;
        int path_iter_index = 0;
        while (path_iter.hasPrevious()){
            msg_commnand_list = path_iter.previous();
            path_iter_index++;

            for (int i = 0; i < msg_commnand_list.length; i++) {
                if(msg_commnand_list[i] == null || msg_commnand_list[i].equals("null")){
                    msg1[i]= "NoOp";
                    System.out.print("#msg_commnand_list was null for agent: " + i +"at path_iter_index: " + path_iter_index );
                    System.out.println("# new message out of null:  " + msg1[i]);

                }else {
                    msg1[i]= msg_commnand_list[i];
                }
            }

            String joinedString =  String.join(";", msg1);
            System.out.print(joinedString);

            String response = serverMessages.readLine();
            //System.out.println("#response from srv:"+response);

            if (response.contains("false")) {
                System.err.println("#Server responsed with: "+ response + " to the inapplicable action: " + joinedString + " action number: " + path_iter_index + "\n");
                break;
            }
        }
    }

    public synchronized static ArrayList<String[]> getPathOperatorDecompositionScheduling() throws Exception {
        LinkedList<Agent> agents_unsolved = MapFixedObjects.getAgents();
        Set<Box> boxes_unsolved = MapFixedObjects.getBoxes();
        /*for (Box box : boxes_unsolved) {
            System.out.println("#boxes_unsolved: " + box.getLetterMark()+" ");
        }*/
        Set<Integer> __boxes_ids = MapFixedObjects.getAllBoxesIds();
        /*for (Integer inttt : __boxes_ids) {
            System.out.println("#__boxes_ids: " + inttt+" ");
        }*/
        Set<Integer> __agents_ids = MapFixedObjects.getAgentsMarks();
        /*for (Integer inttt : __agents_ids) {
            System.out.println("#__agents_ids: " + inttt+" ");
        }*/

        LinkedList<Box> boxes = new LinkedList<>(boxes_unsolved);


        DivideAndScheduleMovables divideAndScheduleMovables = new DivideAndScheduleMovables(__boxes_ids);
        MovablesScheduling movables_scheduling = divideAndScheduleMovables.getAgentsScheduled(agents_unsolved);

        ArrayList<Agent> agents_sched = movables_scheduling.getAgentsScheduled();
        LinkedList<Box> boxes_sched = movables_scheduling.getBoxesScheduled();

        Set<Integer> agents_ids = movables_scheduling.getAgentsIds();
        Set<Integer> boxes_ids = movables_scheduling.getBoxesIds();

        SearchScheduled sched_group = movables_scheduling.getStartGroupAgentsBoxes_ToSearch();

        int[][] _total_group = sched_group.getTotalGroup();
        total_group_copy = Arrays.copyOf(_total_group, _total_group.length);
        for (int row = 0; row < _total_group.length; row++) {
            total_group_copy[row] = Arrays.copyOf(_total_group[row], _total_group[row].length);
        }


        int[] start_group_agents = sched_group.getTotalGroup()[SearchScheduled.START_GROUP_AGENTS];
       /* for (int i = 0; i < start_group_agents.length; i++) {
            System.out.println("#start_group_agents: " + i +" "+ start_group_agents[i] + " ");
        }*/

        int[] start_group_agents123 = sched_group.getTotalGroup()[SearchScheduled.START_GROUP_AGENTS];
        int[] index_agents = sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_AGENTS];
        int[] start_group = sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];
        HashMap<Integer, int[]> agents_idx_to_boxes_idx = sched_group.getAgentstIdxsToBoxesIdxs();


        TrackedGroups trackedGroups = movables_scheduling.getTrackedGroups();
        GroupSearch groupSearch = new GroupSearch(trackedGroups);

        int[] conflicting_group = new int[0];
        int[][][] conflicting_paths = new int[0][][];
        sched_group.setState(SearchScheduled.NEXT_GOAL_TO_BOX);

        SearchTaskResult searchTaskResult = groupSearch.runAgentsSearchMA(sched_group, conflicting_group, conflicting_paths);
        ArrayDeque<int[]> path_found_1 = searchTaskResult.getPath();
        System.out.print("#path_found_1-1.size() :" + path_found_1.size());

        PathProcessing pathProcessing = new PathProcessing();
        pathProcessing.resetTimeSteps(path_found_1);
        divideAndScheduleMovables.setAgentsGoalsFound(searchTaskResult);

        if(path_found_1.size() > 0){
            int[] final_agents_position = path_found_1.peek();
            boolean is_solved_changed = movables_scheduling.setAgentsScheduledSolvedResults(searchTaskResult, final_agents_position, SolvedStatus.GOAL_STEP_SOLVED);
        }else {
            System.out.println("#search failed");
        }
        System.out.print("#path_found_1-2.size() :" + path_found_1.size());

        int[][] _standard_state = new int[1][];
        int[] goal_state = groupSearch.runPreProceesOfGoallState(searchTaskResult, _standard_state);
        int[] standard_state = _standard_state[0];

        ArrayList<String[]> path_output_found_1 = new ArrayList<>();
        if (path_found_1.size() > 0){
            System.out.print("#path_found_1-3.size() :" + path_found_1.size());
            int[] agts_state = searchTaskResult.getPath().pop();
            path_output_found_1 = pathProcessing.getMAAgentMoves(searchTaskResult.getPath(), sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_AGENTS]);

            System.out.println("#path_output_found_1 : " + path_output_found_1.size());

            for (String[] __msg : path_output_found_1){
                System.out.println("#__msg_1:  " + Arrays.toString(__msg));
            }


            for (int i = path_output_found_1.size() - 1; i > path_output_found_1.size() - 4 ; i--) {
                String msg_i = Arrays.toString(path_output_found_1.get(i));
                if (msg_i.equals("NoOp;NoOp;")){
                    System.out.println("#msg111 true true:  " + msg_i);
                }
            }

            //path_output_found = pathProcessing.getMAAgentBoxesMoves(path_found, index_agents, agents_idx_to_boxes_idx);
        }else {
            System.out.println("######################## PATH NOT FOUND######################");
        }
        System.out.println("#path_output_found: " + path_output_found_1.size());





        ///////////////////////////////next search /////////////////////////////////////
        System.out.println("######################## NEXT SEARCH ######################");

        TaskScheduled task_scheduled = divideAndScheduleMovables.getSearchResults();

        SearchScheduled search_sched = movables_scheduling.getMatchedAgentsBoxes(task_scheduled);
        search_sched.setStart_coordinates_agts_boxes(goal_state);
        for (int i = 0; i < search_sched.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP].length; i++) {
            System.out.println("#start_group agents with boxes: " + i +" "+ search_sched.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP][i] + " ");
        }

        int[][] total_group = search_sched.getTotalGroup();


        conflicting_group = new int[0];
        conflicting_paths = new int[0][][];


        SearchTaskResult searchTaskResult2 = groupSearch.runAgentsBoxesSearchMA(search_sched, conflicting_group, conflicting_paths);
        ArrayDeque<int[]> path_found_2 = searchTaskResult2.getPath();

        assert standard_state != null;
        path_found_2.add(standard_state);


        ArrayList<String[]> path_output_found_2 = new ArrayList<>();
        if (path_found_2.size() > 0){
            pathProcessing.resetTimeSteps(path_found_2);
            path_output_found_2 = pathProcessing.getMAAgentBoxesMoves(searchTaskResult2);
        }else {
            System.out.println("########################PATH NOT FOUND######################");
        }

        System.out.println("#path_output_found_2:" + path_output_found_2.size());
        if (path_output_found_2.size() > 0){
            //System.out.println("#path_output_found_2:" + Arrays.toString(path_output_found_2.get(0)));
            for (String[] __msg : path_output_found_2){
                System.out.println("#__msg_2:  " + Arrays.toString(__msg));
            }
        }
        path_output_found_2.addAll(path_output_found_1);


        return path_output_found_2;
    }






    public synchronized static ArrayList<String[]> getPathOperatorDecompositionSchedulingBackUp() throws Exception {
        LinkedList<Agent> agents_unsolved = MapFixedObjects.getAgents();

        DivideAndScheduleMovables divideAndScheduleMovables = new DivideAndScheduleMovables(MapFixedObjects.getAllBoxesIds());
        MovablesScheduling movables_scheduling = divideAndScheduleMovables.getAgentsScheduled(agents_unsolved);

        SearchScheduled sched_group = movables_scheduling.getStartGroupAgentsBoxes_ToSearch();
        int[][] total_group = sched_group.getTotalGroup();
        total_group_copy = Arrays.copyOf(total_group, total_group.length);
        for (int row = 0; row < total_group.length; row++) {
            total_group_copy[row] = Arrays.copyOf(total_group[row], total_group[row].length);
        }

        int[] start_group_agents = sched_group.getTotalGroup()[SearchScheduled.START_GROUP_AGENTS];
        for (int i = 0; i < start_group_agents.length; i++) {
            System.out.println("#start_group_agents: " + i +" "+ start_group_agents[i] + " ");
        }

        TrackedGroups trackedGroups = movables_scheduling.getTrackedGroups();
        GroupSearch groupSearch = new GroupSearch(trackedGroups);

        int[] conflicting_group = new int[0];
        int[][][] conflicting_paths = new int[0][][];


        SearchTaskResult searchTaskResult = groupSearch.runAgentsSearchMA(sched_group, conflicting_group, conflicting_paths);
        ArrayDeque<int[]> path_found_1 = searchTaskResult.getPath();

        PathProcessing pathProcessing = new PathProcessing();
        pathProcessing.resetTimeSteps(path_found_1);
        divideAndScheduleMovables.setAgentsGoalsFound(searchTaskResult);

        if(path_found_1.size() > 0){
            int[] final_agents_position = pathProcessing.getValidAgentsGoalCoordinates(searchTaskResult);
            boolean is_solved_changed = movables_scheduling.setAgentsScheduledSolvedResults(searchTaskResult, final_agents_position, SolvedStatus.GOAL_STEP_SOLVED);
        }

        String[] path_not_found = new String[sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_AGENTS].length];
        for (int i = 0; i < path_not_found.length; i++) {
            path_not_found[i] = "PATH NOT FOUND";
        }

        ArrayList<String[]> path_output_found_1 = new ArrayList<>();
        if (path_found_1.size() > 0){
            path_output_found_1 = pathProcessing.getMAAgentMoves(searchTaskResult.getPath(), sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_AGENTS]);
            System.out.println("#path_output_found_1 : ");

            for (String[] __msg : path_output_found_1){
                System.out.println("#__msg_1:  " + Arrays.toString(__msg));
            }

            String[] msg111 = path_output_found_1.get(path_output_found_1.size() - 1);
            System.out.println("#msg111  msg111 msg111:  " + Arrays.toString(msg111));

            for (int i = path_output_found_1.size() - 1; i > path_output_found_1.size() - 4 ; i--) {
                String msg_i = Arrays.toString(path_output_found_1.get(i));
                if (msg_i.equals("NoOp;NoOp;")){
                    System.out.println("#msg111 true true:  " + msg_i);
                }
            }

            //path_output_found = pathProcessing.getMAAgentBoxesMoves(path_found, index_agents, agents_idx_to_boxes_idx);
        }else {
            System.out.println("######################## PATH NOT FOUND######################");
        }
        System.out.println("#path_output_found: " + path_output_found_1.size());




        ///////////////////////////////next search /////////////////////////////////////
        System.out.println("######################## NEXT SEARCH ######################");

        TaskScheduled task_scheduled = divideAndScheduleMovables.getSearchResults();
        SearchScheduled search_sched = movables_scheduling.getMatchedAgentsBoxes(task_scheduled);

        for (int i = 0; i < search_sched.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP].length; i++) {
            System.out.println("#start_group agents with boxes: " + i +" "+ search_sched.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP][i] + " ");
        }

        SearchTaskResult searchTaskResult2 = groupSearch.runAgentsBoxesSearchMA(search_sched, conflicting_group, conflicting_paths);
        ArrayDeque<int[]> path_found_2 = searchTaskResult2.getPath();

        ArrayList<String[]> path_output_found_2 = new ArrayList<>();
        if (path_found_2.size() > 0){
            pathProcessing.resetTimeSteps(path_found_2);
            path_output_found_2 = pathProcessing.getMAAgentBoxesMoves(searchTaskResult2);
        }else {
            System.out.println("########################PATH NOT FOUND######################");
        }

        System.out.println("#path_output_found_2:" + path_output_found_2.size());
        if (path_output_found_2.size() > 0){
            //System.out.println("#path_output_found_2:" + Arrays.toString(path_output_found_2.get(0)));
            for (String[] __msg : path_output_found_2){
                System.out.println("#__msg_2:  " + Arrays.toString(__msg));
            }
        }

        path_output_found_2.addAll(path_output_found_1);

        return path_output_found_2;
    }

}
