package roadgraph;

import geography.GeographicPoint;

public class AStarVertex extends ComparableVertex {
	private double estToGoal;
	
	public AStarVertex(Vertex v, GeographicPoint goal) {
		super(v);
		estToGoal = getVertex().getPoint().distance(goal);
	}
	
	public double getEstToGoal() {
		return estToGoal;
	}
	
	@Override
	public int compareTo(ComparableVertex other) {
		AStarVertex astar = (AStarVertex) other;
		double first = this.getDistanceFromStart() + estToGoal;
		double second = astar.getDistanceFromStart() + astar.getEstToGoal();
		
		if (first < second) return -1;
		if (first > second) return 1;
		return 0;
		
	}
}
