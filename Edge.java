
public class Edge {
	String name;
	float weight;
	boolean status;
	 public Edge( String nm )
     { name = nm;    status = true;  reset(); }
    
	 public void reset( )
     { weight = graph.INFINITY;}    
}
