#ifndef MAP_H
#define MAP_H

/* A data structure to represent a wall. The wall was declared with the context of being on the right of the "rover".
 * When using the structure, if the rover is parallel to the right wall, use the members as is.
 * If it is parallel to the left wall, take startVertex as the end of the wall and endVertex to be the beginning.
*/
typedef struct Wall {
	struct Wall *prevWall;
	int startVertex[2];
	int length;
	int endVertex[2];
	struct Wall *nextWall;
} WallNode;

extern const int numVerticies;
extern const int verticies[];
extern const int destination[];

#endif