package sysmain;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.SearchClient;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.PathsStoreQuerying;
import org.agents.planning.conflicts.SearchGroupStrategy;
import org.agents.planning.SearchStrategy;
import org.agents.planning.conflicts.ConflictAvoidanceTable;
import org.agents.searchengine.SearchEngineOD;
import org.agents.searchengine.SearchEngineSA;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.DecimalFormat;
import java.util.*;

/**
 * This class is used to lunch project
 *
 * @author autor
 *  {@link ConflictAvoidanceTable.UpdatePathToMain} interface
 *   to handle updates of path
 */
public final class Main implements ConflictAvoidanceTable.UpdatePathToMain{

    private static final DecimalFormat df = new DecimalFormat("0.0000");

    public ConflictAvoidanceTable path; // path will be update in function updateConflictAvoidanceTable()

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

            mapFixedObjects.setUpTrackedMovables(MapFixedObjects.getAgents(),MapFixedObjects.getBoxes());
            PathsStoreQuerying pathsStoreQuerying = new PathsStoreQuerying();
            ConflictAvoidanceTable conflictAvoidanceTable = new ConflictAvoidanceTable(pathsStoreQuerying);
            ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules = new ConflictAvoidanceCheckingRules(conflictAvoidanceTable);

            SearchEngineSA searchEngine = new SearchEngineSA(conflictAvoidanceCheckingRules);
            SearchStrategy searchStrategy = new SearchStrategy(searchEngine);
            ArrayDeque<ListIterator<String>> paths_iterations = searchStrategy.getPathsSequencial(searchEngine);


            outputPathFor(serverMessages,2, 1, paths_iterations.pop());
            outputPathFor(serverMessages,2, 2, paths_iterations.pop());


            long endTime = System.nanoTime();
            String duration = df.format( (endTime - startTime) / 1000000000.0 );
            System.out.println("#Result time duration : " + duration    );
        }


        catch (Throwable e) {
            e.printStackTrace();
        }
    }

    private static void outputPathFor(BufferedReader serverMessages, int slots, int slot_number, ListIterator<String> path_iter) throws IOException {
        String[] msg1 = new String[slots+1];
        Arrays.fill(msg1, "NoOp");
        msg1[slots]=  System.lineSeparator();

        String msg_commnand;
        while (path_iter.hasPrevious()){
            msg_commnand = path_iter.previous();

            msg1[slot_number-1]= msg_commnand;
            String joinedString =  String.join(";", msg1);
            System.out.print(joinedString);

            String response = serverMessages.readLine();
            //System.out.println("#response from srv:"+response);

            if (response.contains("false")) {
                System.err.println("#Server responsed with"+ response + "to the inapplicable action:" + joinedString + "\n");
                break;
            }
        }


    }

    public void updateConflictAvoidanceTable(){
            // update path of type ConflictAvoidanceTable

    }

    public ConflictAvoidanceTable getConflictAvoidanceTable(){
        return path;
    }
}
