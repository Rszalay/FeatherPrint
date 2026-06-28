Newtons method like method for point distribution around an arbitrary closed geometry

1. find the centroid of the perimeter slice at layer
	a. use the layer centroid not the bounding box
2. project 1 ray, representing stringer 1 to the outer perimeter. this is the fixed stringer.
3. for each additional stringer, project 1 ray, equal angular distance from the others. there must be an even number of stringers.
4. create a list of the points where the rays clip to the outer perimeter.
5. create a list of the linear distances between them and average.
6. proceding clockwise starting with distance s2-s1, if the distance from the average, move it up to 360/(a*n) (n the number of stringers) towards average. stringer 1 does not move.
7. go to step 5, 3 times, but increase a by some amount.
8. for each stringer, create a line segment from the perimeter point to the point where the line segment crosses the inner perimeter.
9. subtract off the line width from the ends for the perimeter walls.
10. on next layer, advance fixed stringer by pitch.