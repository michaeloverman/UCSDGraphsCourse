package roadgraph;

import geography.GeographicPoint;

/**
 * MapEdge class
 * Holds start and end points of the edge (road)
 * Keeps name, road type and length
 * (though not using those yet...
 * 
 * @author Michael
 *
 */
public class MapEdge {
	private GeographicPoint start;
	private GeographicPoint end;
	private String name;
	private String type;
	private double length;
	
	/**
	 * Constructor creates an edge
	 * @param GeographicPoint start
	 * @param GeographicPoint end
	 * @param String name
	 * @param String type
	 * @param double length
	 */
	public MapEdge(GeographicPoint start, GeographicPoint end, String name, String type, double l) {
		this.start = start;
		this.end = end;
		this.name = name;
		this.type = type;
		this.length = l;
	}

	/**
	 * Get location of start of edge
	 * @return GeographicPoint
	 */
	public GeographicPoint getStart() {
		return start;
	}

	/**
	 * Set location of start of edge
	 * (probably don't need this, but...)
	 * @param GeographicPoint start
	 */
	public void setStart(GeographicPoint start) {
		this.start = start;
	}

	/**
	 * Get location of end of edge
	 * @return GeographicPoint
	 */
	public GeographicPoint getEnd() {
		return end;
	}

	/**
	 * Set location of end of edge
	 * (probably don't need this...)
	 * @param GeographicPoint end
	 */
	public void setEnd(GeographicPoint end) {
		this.end = end;
	}

	/**
	 * Get name of edge (road)
	 * @return String
	 */
	public String getName() {
		return name;
	}

	/**
	 * Set name of edge
	 * (more likely to need this than setEnd or setStart...)
	 * @param String name
	 */
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * Get road type
	 * @return String
	 */
	public String getType() {
		return type;
	}

	/**
	 * Set road type (in case it changes....?)
	 * @param String type
	 */
	public void setType(String type) {
		this.type = type;
	}

	/**
	 * Get length of road
	 * @return double
	 */
	public double getLength() {
		return length;
	}

	/**
	 * Set length of road
	 * (hard to imagine length of road changing, but just in case)
	 * @param double length
	 */
	public void setLength(double length) {
		this.length = length;
	}
	
	
}
