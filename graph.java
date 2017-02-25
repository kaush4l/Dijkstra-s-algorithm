import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Queue;
import java.util.Scanner;
import java.util.SortedSet;
import java.util.StringTokenizer;
import java.util.TreeSet;


@SuppressWarnings("unused")
public class graph {
	public final static int INFINITY = Integer.MAX_VALUE;
	
	Vertex[] vertices;
//	private MinPriorityQueue queue;		
    public static Map<String,Vertex> vertexMap = new HashMap<String,Vertex>( );
    
    public static Map<String, Boolean> vertexState = new HashMap<String , Boolean>();
    public static Map<String,Edge> EdgeMap = new HashMap<String,Edge>( );
	public static Map<String, HashMap<String, HashMap<Boolean, Float>>> anodes = new HashMap<String, HashMap<String, HashMap<Boolean, Float>>>();
	//anodes = All Connected Nodes

    public void addEdge( String sourceName, String destName, Float weight , Boolean status)
    {
        Vertex v = getVertex( sourceName );
        Vertex w = getVertex( destName );
        Edge e = getEdge( sourceName, destName );
        e.weight = weight;
        v.adj.add( e );
        vertexMap.put(sourceName, v);
        vertexState.put(sourceName, true);
        
        if(vertexState.containsKey(destName)){
        	if(anodes.containsKey(sourceName)){
                HashMap<String, HashMap<Boolean, Float>> outerMap = anodes.get(sourceName);
                HashMap<Boolean, Float> innerMap = new HashMap<Boolean, Float>();
                innerMap.put(status, weight);
                outerMap.put(destName, innerMap);
                anodes.put(sourceName, outerMap);
                } else {
                	Map<Boolean, Float> innerMap = new HashMap<Boolean, Float>();
                	HashMap<String, HashMap<Boolean, Float>> outerMap = new HashMap<String, HashMap<Boolean, Float>>();
                	innerMap.put(status, weight);
                	outerMap.put(destName, (HashMap<Boolean, Float>) innerMap);
                	anodes.put(sourceName, outerMap);
                }
        } else {
        	vertexState.put(destName, status);
        	if(anodes.containsKey(sourceName)){
                HashMap<String, HashMap<Boolean, Float>> outerMap = anodes.get(sourceName);
                HashMap<Boolean, Float> innerMap = new HashMap<Boolean, Float>();
                innerMap.put(status, weight);
                outerMap.put(destName, innerMap);
                anodes.put(sourceName, outerMap);
                } else {
                	Map<Boolean, Float> innerMap = new HashMap<Boolean, Float>();
                	HashMap<String, HashMap<Boolean, Float>> outerMap = new HashMap<String, HashMap<Boolean, Float>>();
                	innerMap.put(status, weight);
                	outerMap.put(destName, (HashMap<Boolean, Float>) innerMap);
                	anodes.put(sourceName, outerMap);
                }
        }        
    }

    public void printPath( String destName )
    {
        Vertex w = vertexMap.get( destName );
        if( w == null )
            throw new NoSuchElementException( "Destination vertex not found" );
        else if( w.dist == INFINITY )
            System.out.println( destName + " is unreachable" );
        else
        {
//            System.out.print( "(Distance is: " + w.dist + ") " );
//            printPath( w );
            System.out.println( );
        }
    }

    /**
     * If vertexName is not present, add it to vertexMap.
     * In either case, return the Vertex.
     */
    private Vertex getVertex( String vertexName )
    {
        Vertex v = vertexMap.get( vertexName );
        if( v == null )
        {
            v = new Vertex( vertexName );
            vertexMap.put( vertexName, v );
        }
        return v;
    }
    
    private Edge getEdge(String source,String dest) {
		 String s= source.concat("_").concat(dest);
		Edge x = EdgeMap.get( s );
	        if( x == null ) {
	            x = new Edge( s );
	            EdgeMap.put( s, x );
	        }
	        
	        return x;
	}

    /**
     * Recursive routine to print shortest path to dest
     * after running shortest path algorithm. The path
     * is known to exist.
     */
    private void printPath( Vertex dest )
    {
        if( dest.prev != null )
        {
            printPath( dest.prev );
            System.out.print( " to " );
        }
        System.out.print( dest.name );
    }
    
    /**
     * Initializes the vertex output info prior to running
     * any shortest path algorithm.
     */
    private void clearAll( )
    {
        for( Vertex v : vertexMap.values( ) )
            v.reset( );
    }

    /**
     * Single-source unweighted shortest-path algorithm.
     */
//    public void unweighted( String startName )
//    {
//        clearAll( ); 
//
//        Vertex start = vertexMap.get( startName );
//        if( start == null )
//            throw new NoSuchElementException( "Start vertex not found" );
//
//        Queue<Vertex> q = new LinkedList<Vertex>( );
//        q.add( start ); start.dist = 0;
//
//        while( !q.isEmpty( ) )
//        {
//            Vertex v = q.remove( );
//
//            for( Vertex w : v.adj )
//            {
//                if( w.dist == INFINITY )
//                {
//                    w.dist = v.dist + 1;
//                    w.prev = v;
//                    q.add( w );
//                }
//            }
//        }
//    }
    

    
    public void deleteedge( String v1 , String v2 ){
    	if(anodes.containsKey(v1)){
    		HashMap<String, HashMap<Boolean, Float>> outerMap = anodes.get(v1);
    		if(outerMap.containsKey(v2)){
    			outerMap.remove(v2);
    			System.out.println("Edge Deleted!");
    			} else {
    				System.out.println("No edge is present to delete");
    				}
    		} else {
    			System.out.println("No such vertex present");
    			}
    }
    
    
    public void edgedown (String v1 , String v2){
    	if(anodes.containsKey(v1)){
    		HashMap<String, HashMap<Boolean, Float>> outerMap = anodes.get(v1);
    		if(outerMap.containsKey(v2)){
    			HashMap<Boolean, Float> innerMap = outerMap.get(v2);
    			if( innerMap.get(true) != null ){
    				Float dist  = innerMap.get(true);
    				innerMap.remove(true);
        			innerMap.put(false, dist);
        			System.out.println("the edge between " + v1 + " " + v2 + " is down.!" );
    			} else {
    				System.out.println("The edge is already down.");
    			}
    			
    		} else {
    			System.out.println("No Edge Present between the given vertices");
    		}
    	} else {
    		System.out.println("The Vertex is not present");
    	}
    }
    
    public void edgeup(String v1, String v2){
    	if(anodes.containsKey(v1)){
    		HashMap<String, HashMap<Boolean, Float>> outerMap = anodes.get(v1);
    		if(outerMap.containsKey(v2)){
    			HashMap<Boolean, Float> innerMap = outerMap.get(v2);
    			Float dist = innerMap.get(false);
    			innerMap.remove(false);
    			innerMap.put(true, dist);
    			System.out.println("Edge between " + v1 + " and " + v2 + " is up.");
    		} else {
    			System.out.println("No Edge Present between the given vertices.");
    		}
    	} else {
    		System.out.println("The given vertex is not present.");
    	}
    }
    
    public void vertexdown(String v1) {
    	if(vertexMap.containsKey(v1)){
    		if(vertexState.get(v1) == true){
    			vertexState.remove(v1);
    			vertexState.put( v1 , false );
    			} else {
    				System.out.println("The vertex is already down.");
    				}
    		} else {
    			System.out.println("No such vertex present.");
    		}
    }
    
    public void vertexup(String v1) {
    	if(vertexMap.containsKey(v1)){
    		if(vertexState.get(v1) == false){
    			vertexState.remove(v1);
    			vertexState.put(v1, true);
    			} else {
    				System.out.println("The vertex is already down.");
    			}
    	} else {
    		System.out.println("No such vertex exists.");
    	}
    }
    
	public void reachable() {
		SortedSet<String> keys = new TreeSet<String>(vertexMap.keySet());
		for (Iterator<String> it = keys.iterator(); it.hasNext();) {
			Vertex v = vertexMap.get(it.next());
			if (vertexState.get(v)) {
				System.out.println(v.name);
				for (Iterator<String> it2 = keys.iterator(); it2.hasNext();) {
					Vertex w = vertexMap.get(it2.next());
					System.out.println(vertexMap.get(w));
				}
			}
		}
	}
    
	@SuppressWarnings("unchecked")
	public void reach ( ) {
		@SuppressWarnings({ "rawtypes" })
		List sortedKeys = new ArrayList(vertexMap.keySet());
    	Collections.sort(sortedKeys);
    	for ( int i = 0 ; i < sortedKeys.size() ; i++ ){
    		if (vertexState.get(sortedKeys.get(i))) {
    		}
    	}
	}
	
    @SuppressWarnings({ "rawtypes", "unchecked" })
	public void print(){
    	//Sort VertexMap
    	List sortedKeys = new ArrayList(vertexMap.keySet());
    	Collections.sort(sortedKeys);
    	
    	for (int i = 0; i < sortedKeys.size(); i++){
    		HashMap<String, HashMap<Boolean, Float>> outerMap = anodes.get(sortedKeys.get(i));
    		
    		if(anodes.get(sortedKeys.get(i)) != null){
    			if(vertexState.get(sortedKeys.get(i)) == true && anodes.containsKey(sortedKeys.get(i))){
        			System.out.println(sortedKeys.get(i));
        			List insideSort = new ArrayList(outerMap.keySet());
        			Collections.sort(insideSort);
        			for(int j = 0; j < insideSort.size(); j++){
        				if(outerMap.get(insideSort.get(j)).get(true) != null ){
        				System.out.println( "     " + insideSort.get(j) + "  " + outerMap.get(insideSort.get(j)).get(true));
        			} else {
        				System.out.println( "     " + insideSort.get(j) + "  " + outerMap.get(insideSort.get(j)).get(false) + " Down" );
        			}
        				}
        		} else {
        			System.out.println(sortedKeys.get(i) + "  Down");
        			List insideSort = new ArrayList(outerMap.keySet());
        			Collections.sort(insideSort);
        			for(int j = 0; j < insideSort.size(); j++){
        				if(outerMap.get(insideSort.get(j)).get(true) != null){
        				System.out.println( "     " + insideSort.get(j) + "  " + outerMap.get(insideSort.get(j)).get(true));
        			} else {
        				System.out.println( "     " + insideSort.get(j) + "  " + outerMap.get(insideSort.get(j)).get(false) + " Down" );
        			}
        				}
        		}
    		}
    	}
    }
    
//    public void shortPath(String source, String dest){
//    	clearAll();
//    	int count = 0;
//    	for( String key : vertexMap.keySet() )
//    		if ( vertexState.get(key) ) {
//    			count++;
//    		}
//    	Vertex s = vertexMap.get(source);
//    	s.dist = 0;
//    	s.prev = null;
//    	Vertex[] queue = new Vertex[count];
//    	int i = 0;
//    	for ( String src : vertexMap.keySet() ) {
//    		if ( vertexState.get( src ) ) {
//    			Vertex v = vertexMap.get(src);
//    			queue[i] = v;
//    			v.pos(i);
//    			i++;
//    		}
//    	}
//    	buildMinHeap(queue , count);
//    	int qsize = count;
//    	while ( qsize != 0 ) {
//    		Vertex u = extractMin ( queue, qsize );
//    		qsize--;
//    		Iterator it = U.adj.listIterator();
//    		while (it.hasNext()) {
//				Edge edge = (Edge) it.next();
//				if (edge.status == true) {
//					String des = edge.getDest();
//					Vertex desti = vertexMap.get(des);
//					if (desti.getAvailability()) {
//						if (desti.dist > (U.dist + edge.getCost())) {
//							double w = U.dist + edge.getCost();
//							desti.dist = w;
//							desti.setPrev(U);
//							int position = desti.getPos();
//							HeapdecreaseKey(Q, position, desti);
//						}
//					}
//				}
//			}
//    	}
//    }
    
//		buildMinHeap(Q, count);
//		int qsize = count;
//		while (qsize != 0) {
//			Vertex U = extractMin(Q, qsize);
//			qsize--;
//			Iterator it = U.adj.listIterator();
//			while (it.hasNext()) {
//				Edge edge = (Edge) it.next();
//				if (edge.getAvailability() == true) {
//					String des = edge.getDest();
//					Vertex desti = vertexMap.get(des);
//					if (desti.getAvailability()) {
//						if (desti.dist > (U.dist + edge.getCost())) {
//							double w = U.dist + edge.getCost();
//							desti.dist = w;
//							desti.setPrev(U);
//							int position = desti.getPos();
//							HeapdecreaseKey(Q, position, desti);
//						}
//					}
//				}
//			}
//		}
//		return vertexMap.get(dest).dist;
//	}
////prints shortest path from source to destination vertex.
//	private void printPath(String desti) {
//		Vertex dest = vertexMap.get(desti);
//		if (dest.prev != null) {
//			printPath(dest.prev.name);
//		}
//		System.out.print(dest.name+" ");
//	}
////HeapdecreaseKey method is called in Dijkstras algorithm after the distance is updated
////for each adjacent vertex. This is to maintain to heap order. 
////Running time of this algorithm is O(lnV)
//	public void HeapdecreaseKey(Vertex[] vertex, int pos, Vertex key) {
//		vertex[pos] = key;
//		while (pos > 0 && vertex[(pos - 1) / 2].dist > vertex[pos].dist) {
//			Vertex temp = vertex[(pos - 1) / 2];
//			Vertex antemp = vertex[pos];
//			temp.setPos(pos);
//			vertex[pos] = temp;
//			antemp.setPos((pos - 1) / 2);
//			vertex[(pos - 1) / 2] = antemp;
//			pos = (pos - 1) / 2;
//		}
//	}
////Builds minimum binary heap
//	public void buildMinHeap(Vertex[] vertex, int index) {
//		int ind = index / 2;
//		for (int j = ind - 1; j >= 0; j--) {
//			// System.out.println(j);
//			minHeapify(vertex, j, index);
//		}
//	}
//	public void minHeapify(Vertex[] vertex, int j, int index) {
//		int l, r, smallest;
//		l = 2 * j + 1;
//		r = 2 * j + 2;
//		if (l < index && vertex[l].dist < vertex[j].dist) {
//			smallest = l;
//		} else
//			smallest = j;
//		if (r < index && vertex[r].dist < vertex[smallest].dist) {
//			smallest = r;
//		}
//		if (smallest != j) {
//			Vertex temp = vertex[smallest];
//			Vertex an = vertex[j];
//			an.setPos(smallest);
//			vertex[smallest] = an;
//			temp.setPos(j);
//			vertex[j] = temp;
//			minHeapify(vertex, smallest, index);
//		}
//	}
////Extracts min element of priority queue
//	public Vertex extractMin(Vertex[] vertex, int size) {
//		int lastind = size - 1;
//		if (size < 1) {
//			System.out.println("Heap Underflow");
//			System.exit(0);
//		}
//		Vertex min = vertex[0];
//		Vertex ano = vertex[lastind];
//		ano.setPos(0);
//		vertex[0] = vertex[lastind];
//		min.setPos(lastind);
//		vertex[lastind] = min;
//		minHeapify(vertex, 0, lastind);
//		return min;
//	}
    
//    public class MinPriorityQueue {
//        Vertex[] queue;
//        int maxSize;
//        int rear = -1, front = -1;
//
//        MinPriorityQueue(int maxSize) {
//            this.maxSize = maxSize;
//            queue = new Vertex[maxSize];
//        }
//
//        public void add(Vertex node) {
//            queue[++rear] = node;
//        }
//
//        public Vertex remove() {
//            Vertex minValuedNode = null;
//            int minValue = Integer.MAX_VALUE;
//            int minValueIndex = -1;
//            front++;
//            for (int i = front; i <= rear; i++) {
//                if (queue[i].state == Vertex.State.IN_Q && queue[i].dist < minValue) {
//                    minValue = queue[i].dist;
//                    minValuedNode = queue[i];
//                    minValueIndex = i;
//                }
//            }
//
//            swap(front, minValueIndex); // this ensures deletion is still done
//                                        // from front;
//            queue[front] = null;// lets not hold up unnecessary references in
//                                // the queue
//            return minValuedNode;
//        }
//
//        public void swap(int index1, int index2) {
//            Vertex temp = queue[index1];
//            queue[index1] = queue[index2];
//            queue[index2] = temp;
//        }
//
//        public boolean isEmpty() {
//            return front == rear;
//        }
//    }
//    
//    public void applyDijkstraAlgorithm(final String sc) {
//    	if( vertexMap.containsKey(sc) ){
//    		Vertex sourceNode = vertexMap.get(sc);
//            queue.add(sourceNode);
//            sourceNode.state = Vertex.State.IN_Q;
//            sourceNode.dist = 0; // cost of reaching Source from Source Node itself
//                                    // is 0, for all others we still need to
//                                    // discover the cost so the cost for them has
//                                    // been already initialized to Integer.MAX_VALUE
//            while (!queue.isEmpty()) {
//                Vertex visitedNode = queue.remove();
//                visitedNode.state = Vertex.State.VISITED;
//                Vertex.Neighbour connectedEdge = visitedNode.neighbourList;
//                while (connectedEdge != null) {
//                    Vertex neighbour = vertices[connectedEdge.index];
//                    // adding the not enqued neighbor nodes in the queue
//                    if (neighbour.state == Vertex.State.NEW) {
//                        queue.add(neighbour);
//                        neighbour.state = Vertex.State.IN_Q;
//                    }
//                    // updating [relaxing] the costs of each non visited neighbor
//                    // node if its
//                    // have been made lesser.
//                    if (neighbour.state != Vertex.State.VISITED && ((connectedEdge.weight + visitedNode.dist) < neighbour.dist)) {
//                        neighbour.dist = connectedEdge.weight + visitedNode.dist;
//                    }
//                    connectedEdge = connectedEdge.next;
//                }
//            }
//            
//            //now printing the shortest distances
//            for(int i = 0; i < vertexMap.size(); i++){
//                if(vertices[i].dist != Integer.MAX_VALUE){
//                    System.out.println("distance from "+sourceNode.name +" to "+vertices[i].name+" is " +vertices[i].dist);
//                }else{
//                    System.out.println(vertices[i].name +" is not reachable from "+sourceNode.name);
//                }
//            }
//    	}
//    }
    
//    public void findShortestPaths(String sc){
//    	
//        applyDikjstraAlgorith(vertexMap.get(sc));
//        Vertex use = vertexMap.get(sc);
//        for(int i = 0; i < INFINITY; i++){
//            System.out.println("Distance of "+use.name+" from Source: "+ use.dist);
//        }
//    }    
//    public static class Heap {
//        private Vertex[] heap;
//        private int maxSize;
//        private int size;
//
//        public Heap(int maxSize) {
//            this.maxSize = maxSize;
//            heap = new Vertex[maxSize];
//        }
//
//        public void add(Vertex u) {
//            heap[size++] = u;
//            heapifyUP(size - 1);
//        }
//
//        public void heapifyUP(Vertex u) {
//            for (int i = 0; i < maxSize; i++) {
//                if (u == heap[i]) {
//                    heapifyUP(i);
//                    break;
//                }
//            }
//        }
//        
//
//        public void heapifyUP(int position) {
//            int currentIndex = position;
//            Vertex currentItem = heap[currentIndex];
//            int parentIndex = (currentIndex - 1) / 2;
//            Vertex parentItem = heap[parentIndex];
//            while (currentItem.compareTo(parentItem) == -1) {
//                swap(currentIndex, parentIndex);
//                currentIndex = parentIndex;
//                if (currentIndex == 0) {
//                    break;
//                }
//                currentItem = heap[currentIndex];
//                parentIndex = (currentIndex - 1) / 2;
//                parentItem = heap[parentIndex];
//            }
//        }
//        
//        public void heapifyDown(int postion) {
//            if (size == 1) { return; }
//            int currentIndex = postion;
//            Vertex currentItem = heap[currentIndex];
//            int leftChildIndex = 2 * currentIndex + 1;
//            int rightChildIndex = 2 * currentIndex + 2;
//            int childIndex;
//            if (heap[leftChildIndex] == null) { return; }
//            if (heap[rightChildIndex] == null) {
//                childIndex = leftChildIndex;
//            } else if (heap[rightChildIndex].compareTo(heap[leftChildIndex]) == -1) {
//                childIndex = rightChildIndex;
//            } else {
//                childIndex = leftChildIndex;
//            }
//            Vertex childItem = heap[childIndex];
//            while (currentItem.compareTo(childItem) == 1) {
//                swap(currentIndex, childIndex);
//                currentIndex = childIndex;
//                currentItem = heap[currentIndex];
//                leftChildIndex = 2 * currentIndex + 1;
//                rightChildIndex = 2 * currentIndex + 2;
//                if (heap[leftChildIndex] == null) {
//                    return;
//                }
//                if (heap[rightChildIndex] == null) {
//                    childIndex = leftChildIndex;
//                } else if (heap[rightChildIndex].compareTo(heap[leftChildIndex]) == -1) {
//                    childIndex = rightChildIndex;
//                } else {
//                    childIndex = leftChildIndex;
//                }
//            }
//        }
//        
//        public void swap(int index1, int index2) {
//            Vertex temp = heap[index1];
//            heap[index1] = heap[index2];
//            heap[index2] = temp;
//        }
//        public boolean isEmpty() {
//            return size == 0;
//        }
//        public Vertex remove() {
//            Vertex v = heap[0];
//            swap(0, size - 1);
//            heap[size - 1] = null;
//            size--;
//            heapifyDown(0);
//            return v;
//        }
//    }
//    
//    public void applyDikjstraAlgorith(String sr) {
//    	
//    	Vertex src =  vertexMap.get(sr);
//    	
//        Heap heap = new Heap(INFINITY);
//        heap.add(src);
//        src.state = Vertex.State.IN_Q;
//        src.dist = 0;
//        while (!heap.isEmpty()) {
//            Vertex u = heap.remove();
//            u.state = Vertex.State.VISITED;
//            Vertex.Neighbour temp = u.nb;
//            while (temp != null) {
//                if (vertices[temp.index].state == Vertex.State.NEW) {
//                    heap.add(vertices[temp.index]);
//                    vertices[temp.index].state = Vertex.State.IN_Q;
//                }
//                if (vertices[temp.index].dist > u.dist + temp.weight) {
//                    vertices[temp.index].dist = u.dist + temp.weight;
//                    heap.heapifyUP(vertices[temp.index]);
//                }
//                temp = temp.next;
//            }
//        }
//    }
    
    
    
    
    /**
     * Process a request; return false if end of file.
     */
    public static boolean processRequest( Scanner in, graph g )
    {
        try
        {
            System.out.print( "Enter start node: " );
            String startName = in.nextLine( ).toLowerCase();

            System.out.print( "Enter destination node: " );
            String destName = in.nextLine( ).toLowerCase();

//            g.unweighted( startName );
//            g.printPath( destName );
        }
        catch( NoSuchElementException e )
          { return false; }
        catch( GraphException e )
          { System.err.println( e ); }
        return true;
    }
    
    /**
     * A main routine that:
     * 1. Reads a file containing edges (supplied as a command-line parameter);
     * 2. Forms the graph;
     * 3. Repeatedly prompts for two vertices and
     *    runs the shortest path algorithm.
     * The data file is a sequence of lines of the format
     *    source destination 
     */
    @SuppressWarnings("resource")
	public static void main(String[] args ) throws FileNotFoundException 
    {
    	boolean status = true;
		String type, sc, dt;
    	Float userv;
    	Scanner in;
        graph g = new graph( );
        try
        {
            // Read the edges and insert
			Scanner s = new Scanner(System.in);
            FileReader fin = new FileReader( args[0] );
            Scanner graphFile = new Scanner( fin );
            String line;
            while( graphFile.hasNextLine( ) )
            {
                line = graphFile.nextLine( );
                StringTokenizer st = new StringTokenizer( line );

                try
                {
                    if( st.countTokens( ) != 3 )
                    {
                        System.err.println( "Skipping ill-formatted line " + line );
                        continue;
                    }
                    String source  = st.nextToken( ).toLowerCase();
                    String dest    = st.nextToken( ).toLowerCase();
                    Float dist    = Float.parseFloat(st.nextToken());
                    
                    g.addEdge( source , dest , dist , status );
                    g.addEdge( dest , source , dist , status );
                }
                catch( NumberFormatException e )
                  { System.err.println( "Skipping ill-formatted line " + line ); }
             }

         }
         catch( IOException e )
           { System.err.println( e ); }

         System.out.println( "File read..." );         
         System.out.println( graph.vertexMap.size( ) + " vertices" );         
         
         in = new Scanner( System.in );
         
         while(true){
        	 System.out.println("Enter any query to process : ");
        	 String inp = in.nextLine().toLowerCase();
        	 StringTokenizer st = new StringTokenizer(inp);
        	 
        	 switch (st.countTokens()){
        	 case 1 :
        		 if (inp.startsWith("print")){
        			 g.print();
        		 } else if (inp.startsWith("reachable")){
        			 g.reachable();
        		 } else if (inp.startsWith("quit")){
        			 System.exit(0);
        		 } else if (inp.startsWith("reach")) {
        			 g.reachable();
        		 } else {
        			 System.out.println("Check query " + inp);
        		 }
        		 break;
        	 case 2 :
        		 if (inp.startsWith("vertexdown")){
        			 type = st.nextToken();
        			 sc = st.nextToken().toLowerCase();
        			 g.vertexdown(sc);
        		 } else if (inp.startsWith("vertexup")){
        			 type = st.nextToken();
        			 sc = st.nextToken().toLowerCase();
        			 g.vertexup(sc); 
        		 } else {
        			 System.out.println("Check query " + inp);
        		 }
        		 break;
        	 case 3 :
        		 if(inp.startsWith("deleteedge")){
        			 type = st.nextToken();
        			 sc = st.nextToken().toLowerCase();
        			 dt = st.nextToken().toLowerCase();
        			 g.deleteedge( sc, dt );
        		 } else if (inp.startsWith("edgedown")){
        			 type = st.nextToken();
        			 sc = st.nextToken().toLowerCase();
        			 dt = st.nextToken().toLowerCase();
        			 g.edgedown( sc, dt );
        		 } else if (inp.startsWith("edgeup")){
        			 type = st.nextToken();
        			 sc = st.nextToken().toLowerCase();
        			 dt = st.nextToken().toLowerCase();
        			 g.edgeup(sc, dt);
        		 } else if (inp.startsWith("path")) {
        			 type = st.nextToken();
        			 sc = st.nextToken().toLowerCase();
        			 dt = st.nextToken().toLowerCase();
        			 //Implement dijkstra
        			 dijkstra dj = new dijkstra( sc, dt, vertexMap );
        			 dj.shortestpath(  );
        		 }else {
        			 System.out.println("Check query " + inp);
        		 }
        		 break;
        	 case 4 :
        		 if(inp.startsWith("addedge")){
        			 type = st.nextToken();
        			 sc = st.nextToken().toLowerCase();
        			 dt = st.nextToken().toLowerCase();
        			 userv = Float.parseFloat(st.nextToken());
     				g.addEdge( sc, dt, userv, status );
        		 }
        		 break;
        		 default :
        			 System.err.println( "Skipping ill-formatted line " + inp );
        	 }
         }
         
    }
}
