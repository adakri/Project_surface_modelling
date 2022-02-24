#ifndef H_EDGE
#define H_EDGE

#include "Polygon.hpp"


class Edge
{
	using VertexType = Vec2;

    private:
        VertexType _v;
        VertexType _w;
        VertexType _reduced_vector; //reduced vector
        bool _isBad = false;
        
    public:
        int _index;
        
        //rule of three
        Edge() = default;
        Edge(const Edge&) = default;
        Edge(Edge&&) = default;
        ~Edge() {};
        Edge(const VertexType &v1, const VertexType &v2);

        //implemented methods
        static bool almost_equal(const Edge &, const Edge &);
        //get line equation from vertex
        void lineFromVexrtex( spec_t &, spec_t &, spec_t &); 
        // Function which converts the input line to its
        // perpendicular bisector. It also inputs the points
        // whose mid-point lies on the bisector
        void perpendicularBisectorFromLine(spec_t &, spec_t &, spec_t &);
        // Returns the intersection point of two lines
        static Vec2 lineLineIntersection(spec_t , spec_t , spec_t ,spec_t , spec_t , spec_t );

        Edge &operator=(const Edge&);
        Edge &operator=(Edge&&) = default;
        bool operator ==(const Edge &e) const;
        friend std::ostream &operator <<(std::ostream &str, const Edge &e);

        //getters and setters
        VertexType get_v() const {return _v;};
        VertexType get_w() const {return _w;};
        bool get_is_Bad() const {return _isBad;};
        void set_is_Bad(bool x){ _isBad = x;};
        VertexType get_reduced_vector() const { return _reduced_vector;};
        double norm();

        //print edge
        virtual void print_edge() const {
            std::cout<< "[" << 
            "[" <<_v.getX() << "," << _v.getY() << "]" <<
            "[" <<_w.getX() << "," << _w.getY() << "]" <<
            "]"<< _index <<
            std::endl;
        };
};

class EdgeBoundary: public Edge
{
    private:
    public:
        std::array<Vec2, 3> _neighbours;
        Edge _edge_vector;
        Eigen::MatrixXf _Gk;

        EdgeBoundary()
        {
        };

        EdgeBoundary( std::array<Vec2, 3> &neighbours, Edge edge_vector)
        {
            _neighbours = neighbours;
            _edge_vector = edge_vector;
        };

        EdgeBoundary( Vec2 &v1, const Vec2 &v2) : Edge(v1, v2)
        {
        };

        void computeGk( std::vector<Vec2> vertices)
        {
            //int vi = _neighbours[0];
            float vix = _neighbours[0].getX();
            float viy = _neighbours[0].getY();
            //int vj = _neighbours[1]; 
            float vjx = _neighbours[1].getX(); 
            float vjy = _neighbours[1].getY();
            //int vl = _neighbours[2]; 
            float vlx = _neighbours[2].getX(); 
            float vly = _neighbours[2].getY();

            _Gk = (
                Eigen::MatrixXf(6, 2) <<
                vix, viy,
                viy, -vix,
                vjx, vjy,
                vjy, -vjx,
                vlx, vly,
                vly, -vlx).finished();
        };
        void print_edge() const {
            std::cout<<"Edge boundary"<<std::endl;
            _edge_vector.print_edge();
            std::cout<<"the neighbours"<<std::endl;
            _neighbours[0].isString();
            _neighbours[1].isString();
            _neighbours[2].isString();
        };
        Eigen::MatrixXf get_Gk() const {return _Gk;};
        bool operator ==(const EdgeBoundary &e) const{
            return this->_edge_vector == e._edge_vector;
        };
};


class EdgeInner : public Edge
{
    private:
    public:
        std::array<Vec2, 4> _neighbours;
        Edge _edge_vector;
        Eigen::MatrixXf _Gk;
        EdgeInner()
        {

        }
        EdgeInner(std::array<Vec2, 4> &neighbours, Edge edge_vector)
        {
            _neighbours = neighbours;
            _edge_vector = edge_vector;
        }

        EdgeInner( Vec2 &v1,  Vec2 &v2) : Edge(v1, v2)
        {
        };

        void computeGk( std::vector<Vec2> &vertices)
        {
            //int vi = _neighbours[0]; 
            float vix = _neighbours[0].getX(); 
            float viy = _neighbours[0].getY();
            //int vj = _neighbours[1]; 
            float vjx = _neighbours[1].getX(); 
            float vjy = _neighbours[1].getY();
            //int vl = _neighbours[2]; 
            float vlx = _neighbours[2].getX(); 
            float vly = _neighbours[2].getY();
            //int vr = _neighbours[3]; 
            float vrx = _neighbours[3].getX();
            float vry = _neighbours[3].getY();

            _Gk = (
                Eigen::MatrixXf(8, 2) <<
                vix, viy,
                viy, -vix,
                vjx, vjy,
                vjy, -vjx,
                vlx, vly,
                vly, -vlx,
                vrx, vry,
                vry, -vrx).finished();
        };
        void print_edge() const {
            std::cout<<"Edge inner"<<std::endl;
            _edge_vector.print_edge();
            std::cout<<"the neighbours"<<std::endl;
            _neighbours[0].isString();
            _neighbours[1].isString();
            _neighbours[2].isString();
            _neighbours[3].isString();
        };
        Eigen::MatrixXf get_Gk() const {return _Gk;};
        bool operator ==(const EdgeInner &e) const{
            return this->_edge_vector == e._edge_vector;
        };
};






#endif