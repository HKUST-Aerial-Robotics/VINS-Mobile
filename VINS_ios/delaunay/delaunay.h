#ifndef H_DELAUNAY
#define H_DELAUNAY

#include "vector2.h"
#include "triangle.h"

#include <vector>

class Delaunay
{
	public:
		const std::vector<Triangle>& triangulate(std::vector<Vec2f_> &vertices);
		const std::vector<Triangle>& getTriangles() const { return _triangles; };
		const std::vector<Edge>& getEdges() const { return _edges; };
		const std::vector<Vec2f_>& getVertices() const { return _vertices; };

	private:
		std::vector<Triangle> _triangles;
		std::vector<Edge> _edges;
		std::vector<Vec2f_> _vertices;
};

#endif
