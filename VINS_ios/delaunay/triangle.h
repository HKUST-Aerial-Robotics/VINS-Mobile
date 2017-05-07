#ifndef H_TRIANGLE
#define H_TRIANGLE

#include "vector2.h"
#include "edge.h"

class Triangle
{
	public:
		Triangle(const Vec2f_ &_p1, const Vec2f_ &_p2, const Vec2f_ &_p3);
	
		bool containsVertex(const Vec2f_ &v);
		bool circumCircleContains(const Vec2f_ &v);
	
		Vec2f_ p1;
		Vec2f_ p2;
		Vec2f_ p3;
		Edge e1;				
		Edge e2;
		Edge e3;
};

inline std::ostream &operator << (std::ostream &str, const Triangle & t)
{
	return str << "Triangle:" << std::endl << "\t" << t.p1 << std::endl << "\t" << t.p2 << std::endl << "\t" << t.p3 << std::endl << "\t" << t.e1 << std::endl << "\t" << t.e2 << std::endl << "\t" << t.e3 << std::endl;
		
}

inline bool operator == (const Triangle &t1, const Triangle &t2)
{
	return	(t1.p1 == t2.p1 || t1.p1 == t2.p2 || t1.p1 == t2.p3) &&
			(t1.p2 == t2.p1 || t1.p2 == t2.p2 || t1.p2 == t2.p3) && 
			(t1.p3 == t2.p1 || t1.p3 == t2.p2 || t1.p3 == t2.p3);
}


#endif
