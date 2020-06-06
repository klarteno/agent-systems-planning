package sysmain;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.SearchClient;
import org.agents.planning.GroupSearch;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.ConflictAvoidanceTable;
import org.agents.planning.schedulling.Synchronization;
import org.agents.planning.schedulling.TrackedGroups;
import org.agents.searchengine.PathProcessing;

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
            
            ArrayList<String[]> pathOperatorDecompositionSearch = getPathOperatorDecompositionSearch();
            ListIterator<String[]> path_iter = pathOperatorDecompositionSearch.listIterator(pathOperatorDecompositionSearch.size());
            ArrayDeque<ListIterator<String[]>> paths_iterations = new ArrayDeque<>();
            paths_iterations.add(path_iter);

            String[] no_of_slots = pathOperatorDecompositionSearch.get(0);
            outputPathsForAgents(serverMessages, no_of_slots.length,  paths_iterations.pop());

            long endTime = System.nanoTime();
            String duration = df.format( (endTime - startTime) / 1000000000.0 );
            System.out.println("#Result time duration : " + duration    );
        }


        catch (Throwable e) {
            e.printStackTrace();
        }
    }

    private static void outputPathsForAgents(BufferedReader serverMessages, int slots_length, ListIterator<String[]> path_iter) throws IOException {
        String[] msg1 = new String[slots_length + 1];
        Arrays.fill(msg1, "NoOp");
        msg1[slots_length]=  System.lineSeparator();

        String[] msg_commnand_list;
        int path_iter_index = 0;
        while (path_iter.hasPrevious()){
            msg_commnand_list = path_iter.previous();
            path_iter_index++;

            for (int i = 0; i < msg_commnand_list.length; i++) {
                if(msg_commnand_list[i] == null){
                    msg1[i]= "NoOp";
                    System.out.println("#msg_commnand_list was null for agent: " + i +"at path_iter_index: " + path_iter_index );
                }

                msg1[i]= msg_commnand_list[i];
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


    public synchronized static ArrayList<String[]> getPathOperatorDecompositionSearch() throws Exception {
        // mapFixedObjects.setUpTrackedMovables(MapFixedObjects.getAgents(), MapFixedObjects.getBoxes());
////////////  scheduling
        Agent agent0 = (Agent) MapFixedObjects.getByMarkNo(0);
        Agent agent1 = (Agent) MapFixedObjects.getByMarkNo(1);
        Box box0 = (Box) MapFixedObjects.getByMarkNo(10);
        int[] coordinates0 = box0.getCoordinates();
        agent0.setGoalPosition(coordinates0);
        Box box1 = (Box) MapFixedObjects.getByMarkNo(11);
        int[] coordinates1 = box1.getCoordinates();
        agent1.setGoalPosition(coordinates1);
/////////////////

        Set<Integer> boxes_ids = MapFixedObjects.getAllBoxesIds();
        Set<Integer> agents_ids = MapFixedObjects.getAgentsMarks();

        ////////////
        TrackedGroups trackedGroups = new TrackedGroups(agents_ids, boxes_ids);
        Synchronization synchronised_time = new Synchronization();
        ConflictAvoidanceCheckingRules avoidanceCheckingRules = new ConflictAvoidanceCheckingRules(trackedGroups, synchronised_time);
        GroupSearch groupSearch = new GroupSearch(avoidanceCheckingRules);

        Collection<? extends Integer> tracked_ids = trackedGroups.getTrackedIds();
        int[] start_group = new int[tracked_ids.size()];
        int index = 0;
        for (Integer id: tracked_ids)
            start_group[index++] = id;

        Set<Integer> index_agents_sched = trackedGroups.getAgentsScheduledIds();
        int[] index_agents = new int[index_agents_sched.size()];
        index = 0;
        for (Integer id: index_agents_sched)
            index_agents[index++] = id;     //int[] index_agents = new int[] {0,1};


        int[] conflicting_group = new int[0];
        int[][][] conflicting_paths = new int[0][][];

        /*
        for (int i = 0; i < start_group.length; i++) {
            System.out.println("start_group" + i +" "+ start_group[i] +" ");
        }
*/


        ArrayDeque<int[]> path_found = groupSearch.runGroupSearchMA(start_group, conflicting_group, conflicting_paths);
        PathProcessing pathProcessing = new PathProcessing();
        pathProcessing.resetTimeSteps(path_found);


        //////////path processing
        //ArrayList<String[]> path_output_found = pathProcessing.getMAAgentMoves(path_found, index_agents);

        HashMap<Integer, int[]> agents_idx_to_boxes_idx = new HashMap<>(); //to refactor this
        agents_idx_to_boxes_idx.put(0, new int[]{2});
        agents_idx_to_boxes_idx.put(1, new int[]{3});


        ArrayList<String[]> path_output_found = pathProcessing.getMAAgentBoxesMoves(path_found, index_agents, agents_idx_to_boxes_idx);

        return path_output_found;
    }







}
