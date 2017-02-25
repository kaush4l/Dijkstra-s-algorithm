
Algorithms and Data Structures

---------------------------------------------------------------------------------------------------------------------------------

	Authors

Name 	:	Kaushal Kanakamedala
Id 	: 	800936486
E-Mail	:	kkanakam@uncc.edu


---------------------------------------------------------------------------------------------------------------------------------

Thanks :
	StackoverFlow.
	java-tips.org
	rosettacode.org

---------------------------------------------------------------------------------------------------------------------------------

Configuration Used :

JAVA Version 	:	1.7.0_79
TextEditor used	:	Brackets
Compiler used 	:	Java compiler
IDE used	:	Eclipse IDE for building project

---------------------------------------------------------------------------------------------------------------------------------

Manifest :
	The folder contains 
 -> graph.java				(The main program to run.)
 -> GraphException.java 	- To handle exceptions
 -> Vertex.java			- This takes care of all the vertices and the list of edges.
 -> dijkstra.java		- Implementation of dijkstra's algorithm.(function : shortestpath)
 -> Readme.txt 			- File explaining all content.

---------------------------------------------------------------------------------------------------------------------------------

Program description :


I used HashMap as accessing data from it is very easy (with key) and fast (if looking for running time o(1)).
I stored data in a HashMap.
The basic structure is HashMap<String 1, HashMap<String 2, HashMap<Boolean, Float>>> which has all information.
The String 1 is key which are all the vertices with edges as key and HashMap as value(Vertex with no edges from them are ignored).
The String 2 is in innerMap is second vertex which are connected from String 1.
So String 2 contains only values of vertex which are connected to String 1.
Another HashMap with String 2 as key and a HashMap as value is implemented.
Here the HashMap has key, value pair as boolean and float.
So we can have any of 2 values (true and false as key). And the distance or time is stored as float.

---------------------------------------------------------------------------------------------------------------------------------

Program :

The basic functions in the program are 
	addedge vertex1 vertex2 time		from v1 to v2 with time, update it present
	
	deleteedge vertex1 vertex2		edge v1 to v2 if present
	edgedown vertex1 vertex2		Mark edge down
	edgeup Vertex1 Vertex2			Mark edge up
	

	vertexdown vertex			Make vertex down
	vertexup vertex				Make vertex up

	reachable				all connected components
	print					print all vertices and edges alphabetically
	quit					to close program


---------------------------------------------------------------------------------------------------------------------------------

Points :

Took care to eliminate all the errors.
The IDE shows no error and the program was run with various test cases and worked fine.
All the input and output are converted to Lower case for easiness of giving query.
Conditions are written to eradicate NullPointerException.
The program has no errors or even suggestions as far as I see in the IDE.

---------------------------------------------------------------------------------------------------------------------------------

Known Bugs :

-> The query typed does not have to be captilized as the program converts it to lower case.
-> So even the printing is shown in lower case letters. (only for easiness of user).
-> Reachable sometimes give NullPointerException.
-> Many methods tried are commented out for future use.

---------------------------------------------------------------------------------------------------------------------------------
