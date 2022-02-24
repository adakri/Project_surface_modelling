#ifndef MESH_H
#define MESH_H


#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>
#include <map>


#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "Triangle.hpp"
#include "earcut.hpp"






using namespace std;

typedef map<int, int, int, int > Vertex_Edge_Graph; 
// edge index, <triangle index, edge index, triangle index, edge index, triangle index, edge index, is_it_on_the_edge>

//mapping the adjacency graph in the vertex/edge lists to each edge index we assowiate the up, down, left, right vertices
//https://www.geeksforgeeks.org/map-associative-containers-the-c-standard-template-library-stl/



//completely abstract 
class Mesh {
    private:

    protected:
        Polygon* _polygon;
    public:
        Mesh() = default;
        Mesh(Polygon* polygon); //add details of mesh parameters
        virtual ~Mesh();
    
}; 


class Delaunay : public Mesh
{
    using VertexType = Vec2;
    using EdgeType = Edge;
    using TriangleType = Triangle;

    private:
        std::vector<TriangleType> _triangles;
        std::vector<EdgeType> _edges;
        
        std::vector<VertexType> _innerPoints;
        int _resolution;
        std::vector<std::vector<int>> _mesh_graph;
        std::map<EdgeType* , std::vector<VertexType*>> _mesh_graph2;
        std::vector<EdgeInner> _edges_inner;
        std::vector<EdgeBoundary> _edges_boundary;
        

        //drawing and GL context
        void setupMesh();
    public:
        std::vector<int> _handlesIndex;
        std::vector<int> _fixedVertices;

        std::vector<VertexType> _vertices;
        //rule of three
        Delaunay() = default;
        Delaunay(const Delaunay&) = default;
        Delaunay(Delaunay&&) = default;
        Delaunay(Polygon*);  
        Delaunay(Polygon*, int);      
        ~Delaunay();


        // implememnted functions
        void triangulate();
        void triangulate_earclip();
        void construct_mesh_graph();
        void construct_mesh_graph2();
        void print_mesh_graph();
        void update();
        void refresh_with_vertices();
        
        //implemented
        void draw_mesh(sf::RenderWindow&);
        void resetTriangles();
        void resetEdges();
        void resetVertices();
        void createInnerPoints();
        double compute_mean_area();


        void print_mesh_triangles() const {
            std::cout<<"====================================================="<<std::endl;
            std::cout<<"The triangles"<<std::endl;
            std::cout<<"{{ \n";
            for(int i=0; i<_triangles.size(); i++)
            {
                _triangles[i].print_triangle();
            }
            std::cout<<"}}"<<std::endl;
        };
        void print_mesh_edges() const {
            std::cout<<"====================================================="<<std::endl;
            std::cout<<"The edges"<<std::endl;
            std::cout<<"====================================================="<<std::endl;
            std::cout<<"{{ \n";
            for(int i=0; i<_edges.size(); i++)
            {
                _edges[i].print_edge();
            }
            std::cout<<"}}"<<std::endl;
        };
        void print_mesh_edges_inner() const {
            std::cout<<"====================================================="<<std::endl;
            std::cout<<"The edges inner"<<std::endl;
            std::cout<<"====================================================="<<std::endl;
            std::cout<<"{{ \n";
            for(int i=0; i<_edges_inner.size(); i++)
            {
                _edges_inner[i].print_edge();
            }
            std::cout<<"}}"<<std::endl;
        };
        void print_edges_boundary() const {
            std::cout<<"====================================================="<<std::endl;
            std::cout<<"The _edges boundary"<<std::endl;
            std::cout<<"====================================================="<<std::endl;
            std::cout<<"{{";
            for(int i=0; i<_edges_boundary.size(); i++)
            {
                _edges_boundary[i].print_edge();
            }
            std::cout<<"}}"<<std::endl;
        };
        void print_mesh_vertices() const {
            std::cout<<"====================================================="<<std::endl;
            std::cout<<"The vertices"<<std::endl;
            std::cout<<"{{ \n";
            for(int i=0; i<_vertices.size(); i++)
            {
                _vertices[i].isString();
            }
            std::cout<<"}}"<<std::endl;
        };
        void detail_mesh()
        {
            std::cout<<"====================================================="<<std::endl;
            std::cout<<"Printing the mesh"<<std::endl;
        };



        //getters and setters
        std::vector<TriangleType> getTriangles() {return _triangles;};
        void setTriangles(std::vector<TriangleType> newTriangles) {_triangles = newTriangles;};
        std::vector<EdgeBoundary> get_edges_boundary() const {return _edges_boundary;};
        std::vector<EdgeInner> get_edges_inner() const {return _edges_inner;};
        std::vector<VertexType> getInnerPoints() const {return _innerPoints;};
        std::vector<EdgeType> getEdges() const {return _edges;};
        std::vector<VertexType> getVertices() {return _vertices;};
        std::vector<int> getHandlesIndex(){return _handlesIndex;};
        void setHandlesIndex(int i){_handlesIndex.push_back(i);};
        std::vector<int> getFixedVertices(){return _fixedVertices;};
        void setFixedVertices(int i){_fixedVertices.push_back(i);};
        Polygon* getPolygon(){return _polygon;};
        const int getResolution() const {return _resolution;};
        const std::vector<std::vector<int>> get_mesh_graph() const { return _mesh_graph;};
        

        //operateurs pour surcharge
        Delaunay& operator=(const Delaunay&);
        Delaunay& operator=(Delaunay&&) = default;
};

//shader converter
struct shaderReader
{
    shaderReader(std::string);
    std::string _source;
};



#endif