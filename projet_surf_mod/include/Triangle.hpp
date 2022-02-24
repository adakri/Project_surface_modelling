#ifndef H_TRIANGLE
#define H_TRIANGLE

#include "Edge.hpp"

using VertexType = Vec2;
using EdgeType = Edge;

class Triangle
{
    private:
        VertexType _a ;
        VertexType _b ;
        VertexType _c ;
         // could not const it or size it ?
        EdgeType _e1 = Edge(_a, _b);
        EdgeType _e2 = Edge(_b, _c);
        EdgeType _e3 = Edge(_c, _a);
        VertexType _CircumCenter ;
        spec_t _CircumRadius;
        bool _isBad = false;
        int _index;
    public:
        //rule of three
        Triangle() = default;
        Triangle(const Triangle&) = default;
        Triangle(Triangle&&) = default;
        Triangle(const VertexType, const VertexType , const VertexType );
        ~Triangle(){};
        //implemented methods
        static bool almost_equal(const Triangle &, const Triangle &);
        bool containsVertex(const VertexType &);
        void findCircumCenter();
        bool Cercle_Contains(const VertexType &) const;
        VertexType getCenter(); 
        VertexType third_vertex(VertexType, VertexType);
        double compute_surface();
        void check_validity();
        bool contains_point(Vec2);
        void Barycentric(Vec2, float&, float&, float&);
        

        //This was new to me
        // https://en.cppreference.com/w/cpp/language/rule_of_three
        Triangle &operator=(const Triangle&);
        Triangle &operator=(Triangle&&) = default;
        bool operator ==(const Triangle &t) const;
        friend std::ostream &operator <<(std::ostream &str, const Triangle &t);

        //getters and setters
        VertexType get_a() const {return _a;};
        VertexType get_b() const {return _b;};
        VertexType get_c() const {return _c;};
        EdgeType get_e1() const {return _e1;};
        EdgeType get_e2() const {return _e2;};
        EdgeType get_e3() const {return _e3;};
        bool get_is_Bad() const {return _isBad;};
        VertexType get_CircumCenter() const {return _CircumCenter;};
        spec_t get_CircumRadius() const {return _CircumRadius;};
        int get_index() const { return _index;};

        void set_is_Bad(bool x){ _isBad = x;};
        void set_index(int i) { _index = i;};


        void print_triangle() const {
            std::cout<< "[" << 
            "[" <<_a.getX() << "," << _a.getY() << "]" <<
            "[" <<_b.getX() << "," << _b.getY() << "]" <<
            "[" <<_c.getX() << "," << _c.getY() << "]" <<
            "]"<< _index <<
            std::endl;
        };
};





#endif