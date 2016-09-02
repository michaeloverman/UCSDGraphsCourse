/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private HashMap<GeographicPoint, Vertex> graph;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		graph = new HashMap<>();
		numEdges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return graph.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		Set<GeographicPoint> list = new HashSet<>();
		for (GeographicPoint gp : graph.keySet()) {
			list.add(gp);
		}
		return list;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (location == null) return false;
		if (graph.containsKey(location)) return false;
		graph.put(location, new Vertex(location));
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		// Check all arguments for proper format
		if (!graph.containsKey(from) || !graph.containsKey(to)) {
			throw new IllegalArgumentException("To or From point not in map");
		}
		if (from == null || to == null || roadName == null || roadType == null) {
			throw new IllegalArgumentException("Arguments must not be null");
		}
		if (length < 0) throw new IllegalArgumentException("Road length must not be < 0");
		
		// Create new MapEdge, add it to from's vertex's list of neighbors
		graph.get(from).addNeighbor(new MapEdge(from, to, roadName, roadType, length));
		
		numEdges++;
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null) {
			System.out.println("Start or goal is null. No path exists");
			return null;
		}
		
		// Using Vertex to keep track of things, because they hold the attached edges
		HashMap<Vertex, Vertex> parentMap = new HashMap<>();
		boolean found = bfsSearch(start, goal, parentMap, nodeSearched);
		
		if (!found) {
			return null;
		}
		
		return constructPath(start, goal, parentMap);
	}
	
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
			HashMap<Vertex, Vertex> parentMap) {
		LinkedList<GeographicPoint> list = new LinkedList<>();
		
		Vertex curr = graph.get(goal);
		while(curr != graph.get(start)) {
			list.addFirst(curr.getPoint());
			curr = parentMap.get(curr);
		}
		list.addFirst(start);
		
//		System.out.println(list.toString());
		return list;
	}
	
	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal, 
			HashMap<Vertex, Vertex> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
		// Use GeographicPoint for keeping track of visited nodes, because
		// the start and end points, and Consumer use GeographicPoints
		HashSet<GeographicPoint> visited = new HashSet<>();
		
		// Use Vertices to track what to explore next, because they hold the attached edges/neighbors
		// LinkedList for Queue (FIFO) of nodes to explore
		LinkedList<Vertex> toExplore = new LinkedList<>();
		toExplore.add(graph.get(start));
		boolean found = false;
		
		while(!toExplore.isEmpty()) {
			Vertex curr = toExplore.remove();
			nodeSearched.accept(curr.getPoint());
			if (curr.getPoint().equals(goal)) {
				found = true;
				break;
			}
			for (GeographicPoint gp : curr.getNeighbors()) {
				if (!visited.contains(gp)) {
					visited.add(gp);
					parentMap.put(graph.get(gp), curr);
					toExplore.add(graph.get(gp));
				}
			}
			
			
		}
		
		return found;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null) {
			System.out.println("Start or goal is null. No path exists");
			return null;
		}
		
		
		// Using Vertex to keep track of things, because they hold the attached edges
		HashMap<Vertex, Vertex> parentMap = new HashMap<>();
		
		boolean found = dijkstraSearch(start, goal, parentMap, nodeSearched);
		
		if (!found) {
			return null;
		}
		
		return constructPath(start, goal, parentMap);
		
	}
	
	static int dijkstraSearches;
	private boolean dijkstraSearch(GeographicPoint start, GeographicPoint goal, 
			HashMap<Vertex, Vertex> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
		dijkstraSearches = 0;
		// HashMap to track distanceTo each node. Initialize all to positive infinity
		HashMap<GeographicPoint, ComparableVertex> distanceTo = new HashMap<>();
		for (GeographicPoint gp : graph.keySet()) {
			distanceTo.put(gp, new ComparableVertex(graph.get(gp)));
		}
		
		HashSet<GeographicPoint> visited = new HashSet<>();
		
		Queue<ComparableVertex> toExplore = new PriorityQueue<>();
		ComparableVertex curr = distanceTo.get(start);
		curr.setDistanceFromStart(0.0);
		toExplore.add(curr);
		
		boolean found = false;
		
		
		while(!toExplore.isEmpty()) {
			curr = toExplore.remove();
			System.out.println("DIJKSTRA visiting " + curr.getVertex().toStringWithNeighbors());
			nodeSearched.accept(curr.getPoint());
			dijkstraSearches++;
			if (!visited.contains(curr.getPoint())) {
				visited.add(curr.getPoint());
				if (curr.getPoint().equals(goal)) {
					found = true;
					break;
				}
				
				
				for (MapEdge edge : curr.getEdges()) {
					ComparableVertex endVertex = distanceTo.get(edge.getEnd());
					if (!visited.contains(edge.getEnd()) &&
							((curr.getDistanceFromStart() + edge.getLength()) < endVertex.getDistanceFromStart()) ) {
						endVertex.setDistanceFromStart(curr.getDistanceFromStart() + edge.getLength());
						parentMap.put(endVertex.getVertex(), curr.getVertex());
						toExplore.add(endVertex);
					}
				}
				
			}
			
		}
		
		return found;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null) {
			System.out.println("Start or goal is null. No path exists");
			return null;
		}
		
		
		// Using Vertex to keep track of things, because they hold the attached edges
		HashMap<Vertex, Vertex> parentMap = new HashMap<>();
		
		boolean found = doAStar(start, goal, parentMap, nodeSearched);
		
		if (!found) {
			return null;
		}
		
		return constructPath(start, goal, parentMap);
		
	}

	static int aStarSearches;
	private boolean doAStar(GeographicPoint start, GeographicPoint goal, 
			HashMap<Vertex, Vertex> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
		aStarSearches = 0;
		// HashMap to track distanceTo each node. Initialize all to positive infinity
		HashMap<GeographicPoint, AStarVertex> distanceTo = new HashMap<>();
		for (GeographicPoint gp : graph.keySet()) {
			distanceTo.put(gp, new AStarVertex(graph.get(gp), goal));
		}
		
		HashSet<GeographicPoint> visited = new HashSet<>();
		
		Queue<AStarVertex> toExplore = new PriorityQueue<>();
		AStarVertex curr = distanceTo.get(start);
		curr.setDistanceFromStart(0.0);
		toExplore.add(curr);
		
		boolean found = false;
		
		
		while(!toExplore.isEmpty()) {
			
			curr = toExplore.remove();
//			System.out.println("A* visiting " + curr.getVertex().toStringWithNeighbors());
			
			nodeSearched.accept(curr.getPoint());
			aStarSearches++;
			if (!visited.contains(curr.getPoint())) {
				visited.add(curr.getPoint());
				if (curr.getPoint().equals(goal)) {
					found = true;
					break;
				}
				
				
				for (MapEdge edge : curr.getEdges()) {
					AStarVertex  endVertex = distanceTo.get(edge.getEnd());
					if (!visited.contains(edge.getEnd()) &&
							((curr.getDistanceFromStart() + edge.getLength()) < endVertex.getDistanceFromStart()) ) {
						endVertex.setDistanceFromStart(curr.getDistanceFromStart() + edge.getLength());
						parentMap.put(endVertex.getVertex(), curr.getVertex());
						toExplore.add(endVertex);
					}
				}
				
			}
			
		}
		
		return found;
	}
	
	/**
	 * Takes list of points for stops, assumes first is 'home'
	 * @param stops - points to visit on path
	 * @return - complete list of route to visit all stops
	 */
	public List<GeographicPoint> tsp(List<GeographicPoint> stops) {
		GeographicPoint home = stops.remove(0);
		LinkedList<GeographicPoint> path = new LinkedList<>();
		path.add(home);
		
		while(stops.size() > 0) {
			LinkedList<GeographicPoint> shortestPath = new LinkedList<>();
			double shortestDistance = Double.POSITIVE_INFINITY;
			GeographicPoint shortestEnd = home;
			for (GeographicPoint gp : stops) {
				List<GeographicPoint> curr = aStarSearch(path.get(path.size()-1), gp);
				double shortD = getPathLength(curr);
				if(shortD < shortestDistance) {
					shortestDistance = shortD;
					shortestPath.clear();
					shortestPath.addAll(curr);
					shortestEnd = shortestPath.getLast();
				}
			}
			path.addAll(shortestPath);
			stops.remove(shortestEnd);
		}
		List<GeographicPoint> tripHome = aStarSearch(path.get(path.size()-1), home);
		tripHome.remove(0);
		path.addAll(tripHome);
		
		return path;
	}
	
	private double getPathLength(List<GeographicPoint> path) {
		double distance = 0.0;
		GeographicPoint prev = path.remove(0);
		for(GeographicPoint gp : path) {
			Vertex v = graph.get(prev);
			MapEdge me = v.getEdgeTo(gp);
			if (me == null) distance += 0.0;
			else distance += me.getLength();
			prev = gp;
		}
		
		return distance;
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/mysimplemap.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
//		System.out.println(firstMap.toString());
//		GeographicPoint one = new GeographicPoint(0.0, 0.0);
//		System.out.println("GP one: " + one.toString());
//		GeographicPoint two = new GeographicPoint(0.5, 1.0);
//		System.out.println("GP two: " + two.toString());
//		System.out.println(firstMap.getVertices().toString());
//		Vertex v1 = firstMap.graph.get(one);
//		Vertex v2 = firstMap.graph.get(two);
//		System.out.println(v1.toStringWithNeighbors());
//		Vertex three = firstMap.graph.get(new GeographicPoint(1.5, 1.5));
//		Vertex four = firstMap.graph.get(new GeographicPoint(1.0, 1.0));
//		System.out.println(firstMap.bfs(v1.getPoint(), v2.getPoint()));
//		System.out.println(firstMap.bfs(three.getPoint(), four.getPoint()));
//		System.out.println(firstMap.bfs(v1.getPoint(), three.getPoint()));
//		System.out.println(firstMap.bfs(v2.getPoint(), four.getPoint()));
//		
//		System.out.println("Checking Vertex Methods:");
//		System.out.println("Adding null edge: " + v1.addNeighbor(null));
//		System.out.println("Adding existing edge: " + v1.addNeighbor(new MapEdge(two, four.getPoint(),
//				"pinky", "connector", 1.1)));
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println("Dijkstra: " + dijkstraSearches + "; A*: " + aStarSearches);
		
		System.out.println("Testing TSP routine (simple greedy algorithm)");
		List<GeographicPoint> stops = new LinkedList<>();
		stops.add(testStart);
		stops.add(testEnd);
		stops.add(new GeographicPoint(4.0, -1.0));
		stops.add(new GeographicPoint(7.0, 3.0));
		stops.add(new GeographicPoint(6.5, 0.0));
		System.out.println("Stops:");
		System.out.println(stops.toString());
		List<GeographicPoint> route = simpleTestMap.tsp(stops);
		System.out.println("Route:");
		System.out.println(route.toString());
		System.out.println("Distance: " + simpleTestMap.getPathLength(route));
		
		
//		
//		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
//		
//		// A very simple test using real data
//		testStart = new GeographicPoint(32.869423, -117.220917);
//		testEnd = new GeographicPoint(32.869255, -117.216927);
//		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		System.out.println("Dijkstra: " + dijkstraSearches + "; A*: " + aStarSearches);
//		
//		// A slightly more complex test using real data
//		testStart = new GeographicPoint(32.8674388, -117.2190213);
//		testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		System.out.println("Dijkstra: " + dijkstraSearches + "; A*: " + aStarSearches);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
//		System.out.println("DONE.");
//
//		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
//		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
//		
//		
//		List<GeographicPoint> route = theMap.dijkstra(start,end);
//		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
//
//		System.out.println("Dijkstra: " + dijkstraSearches + "; A*: " + aStarSearches);

		
	}
	
}
