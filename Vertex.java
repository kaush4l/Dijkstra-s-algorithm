import java.util.LinkedList;

public class Vertex {
	public String     name;   // Vertex name
    public LinkedList<Edge> adj;    // Adjacent vertices
    public Vertex     prev;   // Previous vertex on shortest path
    public float        dist;   // Distance of path
    
	public int pos;
	public boolean status;
//    Neighbour neighbourList;
//	  State state;
	
    public Vertex( String nm )
      { name = nm; adj = new LinkedList<Edge>( ); status = true; reset( ); }

    public void reset( )
      { dist = graph.INFINITY; prev = null; }
    
//    public enum State {NEW, IN_Q, VISITED}
    
    public int compareTo(Vertex v) {
        if (this.dist == v.dist) {
            return 0;
        }
        if (this.dist < v.dist) {
            return -1;
        }
        return 1;
    }
//    public class Neighbour {
//        int index;
//        Neighbour next;
//        int weight;
//
//        Neighbour(int index, int weight, Neighbour next) {
//            this.index = index;
//            this.next = next;
//            this.weight = weight;
//        }
//    }
}