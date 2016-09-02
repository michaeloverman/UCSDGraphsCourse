package roadgraph;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;
/**
 * Vertex class (MapNode)
 * holds GeographicPoint marking location
 * holds List of MapEdges to neighboring Vertices
 * 
 * @author Michael
 *
 */
public class Vertex {
	private GeographicPoint location;
	private ArrayList<MapEdge> neighbors;
	
	public Vertex(GeographicPoint loc) {
		this.location = loc;
		this.neighbors = new ArrayList<>();
	}
	
	// Overloaded constructor, in case need to create a Vertex from coordinates
	public Vertex(double latitude, double longitude) {
		this(new GeographicPoint(latitude, longitude));
	}
	
	/**
	 * Get the GeographicPoint marking the location of this Vertex
	 * @return GeographicPoint
	 */
	public GeographicPoint getPoint() {
		return location;
	}
	
	/**
	 * Add a MapEdge denoting a connection (road) to a neighboring Vertex
	 * MapEdge must not be null; must have THIS Vertex as start point, and
	 * must not already be in the list.
	 * @param MapEdge edge
	 * @return boolean of success of addition
	 */
	public boolean addNeighbor(MapEdge edge) {
		if (edge == null) return false;
		if (edge.getStart() != location) return false;
		if (neighbors.contains(edge)) return false;
		neighbors.add(edge);
		return true;
	}
	
	/**
	 * Get neighboring GeographicPoints
	 * @return List<GeographicPoint>
	 */
	public List<GeographicPoint> getNeighbors() {
		List<GeographicPoint> list =  new LinkedList<>();
		for (MapEdge edge : neighbors) {
			list.add(edge.getEnd());
		}
		return list;
		
	}
	
	public List<MapEdge> getEdges() {
		List<MapEdge> list = new LinkedList<>();
		for (MapEdge edge : neighbors) {
			list.add(edge);
		}
		return list;
	}
	
	/**
	 * Use the GeographicPoint's toString() method
	 */
	public String toString() {
		return location.toString();
	}
	
	/**
	 * Extra toString() method, including neighbors, for testing purposes
	 * @return
	 */
	public String toStringWithNeighbors() {
		StringBuffer sb = new StringBuffer(this.toString());
		sb.append("intersects streets: ");
		for (MapEdge edge : neighbors) {
			sb.append(edge.getName() + ", ");
		}
		return sb.toString();
	}
	
	public MapEdge getEdgeTo(GeographicPoint end) {
		for (MapEdge e : neighbors) {
			if (e.getEnd().equals(end)) {
				return e;
			}
		}
		return null;
	}
}
