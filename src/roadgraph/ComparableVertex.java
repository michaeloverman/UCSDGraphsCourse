package roadgraph;

import java.util.List;

import geography.GeographicPoint;

public class ComparableVertex implements Comparable<ComparableVertex> {
	private Vertex vertex;
	private double distanceFromStart;
	
	public ComparableVertex(Vertex v) {
		vertex = v;
		distanceFromStart = Double.POSITIVE_INFINITY;
	}

	public double getDistanceFromStart() {
		return distanceFromStart;
	}

	public void setDistanceFromStart(double distanceFromStart) {
		this.distanceFromStart = distanceFromStart;
	}
	
	public Vertex getVertex() {
		return vertex;
	}

	@Override
	public int compareTo(ComparableVertex other) {
		if (distanceFromStart < other.distanceFromStart) return -1;
		if (distanceFromStart > other.distanceFromStart) return 1;
		return 0;
	}
	
	public GeographicPoint getPoint() {
		return vertex.getPoint();
	}
	
	public List<GeographicPoint> getNeighbors() {
		return vertex.getNeighbors();
	}
	
	public List<MapEdge> getEdges() {
		return vertex.getEdges();
	}
}
