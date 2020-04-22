package sysmain;



import ProjectUtils.Serialization;
import org.agents.Color;
 import org.agents.SearchClient;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.Vector;

public class Main {

    private static final DecimalFormat df = new DecimalFormat("0.0000");

    public static void main(String[] args)
    {
        try
        {
            long startTime = System.nanoTime();

            BufferedReader serverMessages = new BufferedReader(new InputStreamReader(System.in));
            // Use stderr to print to console
            System.err.println("SearchClient initializing. I am sending this using the error output stream.");

            // Read level and create the initial state of the problem
            SearchClient client = new SearchClient(serverMessages);
            client.parse();


            //System.out.println("testing testing .... :  " + client.toString());

            long endTime = System.nanoTime();

            String duration = df.format( (endTime - startTime) / 1000000000.0 );

            System.err.println("Result time duration : " + duration    );


        }
        catch (Throwable e) {
            e.printStackTrace();
        }
    }
}
