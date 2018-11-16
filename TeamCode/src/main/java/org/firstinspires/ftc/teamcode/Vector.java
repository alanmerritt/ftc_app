package org.firstinspires.ftc.teamcode;

/**
 * Vectors are one of the most basic and important principles of movement. This
 * is a vector class that stores and manipulates a vector value with both
 * direction and magnitude (oh yeah!).
 *
 * @author MerrittAM
 * @version 1.3
 * @since 1.0
 */
public class Vector {
	
	// x and y values of the vector
	public double x;
	public double y;
	
	/**
	 * Default constructor; sets the initial values to 0.
	 *
	 * @since 1.0
	 */
	public Vector() {
		
		this.x = 0;
		this.y = 0;
		
	}
	
	/**
	 * Sets the values to the input.
	 *
	 * @since 1.0
	 * @param x
	 *            The x value of the vector.
	 * @param y
	 *            The y value of the vector.
	 */
	public Vector(double x, double y) {
		
		this.x = x;
		this.y = y;
		
	}
	
	/**
	 * Adds the value of a vector.
	 *
	 * @param v
	 *            The vector to add.
	 * @since 1.0
	 */
	public void add(Vector v) {
		
		x += v.x;
		y += v.y;
		
	}
	
	/**
	 * Adds two vectors together and returns the result.
	 *
	 * @param v1
	 *            The first vector to add.
	 * @param v2
	 *            The second vector to add.
	 * @return The sum of the two vectors.
	 */
	public static Vector add(Vector v1, Vector v2) {
		
		return new Vector(v1.x + v2.x, v1.y + v2.y);
		
	}
	
	/**
	 * Subtracts the value of a vector.
	 *
	 * @param v
	 *            The vector to subtract from this one.
	 * @since 1.0
	 */
	public void sub(Vector v) {
		
		x -= v.x;
		y -= v.y;
		
	}
	
	/**
	 * Subtracts one vector from another.
	 *
	 * @param v1
	 *            The vector to subtract from.
	 * @param v2
	 *            The vector to subtract.
	 * @return The new vector value.
	 */
	public static Vector sub(Vector v1, Vector v2) {
		
		return new Vector(v1.x - v2.x, v1.y - v2.y);
		
	}
	
	/**
	 * "Scales" the value of the vector by the input.
	 *
	 * @param d
	 *            The value to multiply the vector by.
	 * @since 1.0
	 */
	public void mult(double d) {
		
		x *= d;
		y *= d;
		
	}
	
	/**
	 * Divides the vector by the input.
	 *
	 * @param d
	 *            The value to divide by.
	 * @since 1.0
	 */
	public void div(double d) {
		
		// make sure that it doesn't divide by 0
		if (d != 0) {
			x /= d;
			y /= d;
		}
		
	}
	
	/**
	 * Returns the magnitude of the vector.
	 *
	 * @return The magnitude.
	 * @since 1.0
	 */
	public double mag() {
		
		return Math.sqrt(x * x + y * y);
		
	}
	
	/**
	 * Normalizes the vector by setting it to a unit length of 1.
	 *
	 * @since 1.0
	 */
	public void normalize() {
		
		double m = mag();
		if (m != 0) {
			x /= m;
			y /= m;
		}
		
	}
	
	/**
	 * Returns the distance between this vector and another.
	 *
	 * @param v
	 *            The vector to compare to.
	 * @return The distance between them
	 * @since 1.1
	 */
	public double dist(Vector v) {
		// distance calculations
		return Math.sqrt((x - v.x) * (x - v.x) + (y - v.y) * (y - v.y));
	}
	
	/**
	 * Returns the distance between this vector and a point.
	 *
	 * @param _x
	 *            The x position of the point.
	 * @param _y
	 *            The y position of the point.
	 * @return The distance between them.
	 */
	public double dist(double _x, double _y) {
		// distance calculations
		return Math.sqrt((x - _x) * (x - _x) + (y - _y) * (y - _y));
	}
	
	public static double dist(Vector v1, Vector v2)
	{
		
		return Math.sqrt((v1.x-v2.x)*(v1.x-v2.x) + (v1.y-v2.y)*(v1.y-v2.y));
		
	}
	
	/**
	 * Calculates the angle the vector is facing.
	 *
	 * @return The angle of the vector.
	 */
	public double heading() {
		
		return Math.atan2(y, x);
		
	}
	
	/**
	 * Returns a copy of the vector.
	 *
	 * @return A copy of the vector.
	 */
	public Vector copy() {
		return new Vector(x, y);
	}
	
	/**
	 * Limits the magnitude to the input value.
	 *
	 * @param max
	 *            The maximum magnitude of the vector.
	 * @since 1.1.2
	 */
	public void limit(double max) {
		
		if ((x * x) + (y * y) > max * max) {
			normalize();
			mult(max);
		}
		
	}
	
	
	/**
	 * Returns a string representation of the vector coordinates.
	 */
	public String toString() {
		return "[" + String.format("%.2f", x) + ", " + String.format("%.2f", y) + "]";
	}
	
}
