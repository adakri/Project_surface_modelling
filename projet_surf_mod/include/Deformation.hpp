#ifndef DEFORMATION_H
#define DEFORMATION_H

#include "Mesh.hpp"
#include "solver.hpp"


//Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

const float w = 1000.0f;
static const Eigen::MatrixXf constantM_boundary =(
	Eigen::MatrixXf(2, 6) <<
	-1, 0, 1, 0, 0, 0,
	0, -1, 0, 1, 0, 0).finished();
static const Eigen::MatrixXf constantM_inner = (
	Eigen::MatrixXf(2, 8) <<
	-1, 0, 1, 0, 0, 0, 0, 0,
	0, -1, 0, 1, 0, 0, 0, 0).finished();
struct HANDLE {
    float hand_index;
    float deformation_en_x;
    float deformation_en_y;
};

class Deformation
{
    private:
        Polygon _polygon; 
        
        std::vector<VertexType> _points;
        std::vector<VertexType> _innerPoints;
        std::vector<EdgeInner> _edges_inner;
        std::vector<EdgeBoundary> _edges_boundary;

        std::vector<VertexType> _fixed_vec;
        std::vector<VertexType> _handles_vec;
        std::vector<VertexType> _handles_vec_deform;

        std::vector<std::vector<float>> _fixed_vec_bary;
        std::vector<std::vector<float>> _handles_vec_bary;
        
        lsq_solver* _solver;

        std::vector<HANDLE> _handles; //for now the distance is fixed (constraints) otherwise we should use a matrixXd
        std::vector<int> _fixed_vertices;
        int _N_meshp, _N_handles, _N_fix;
        int _resolution;
    public:
        Delaunay _mesh;


        Deformation() = default;
        Deformation(Polygon &, Delaunay &, std::vector<HANDLE>, std::vector<int>);
        Deformation(Polygon &, std::vector<HANDLE>, std::vector<int>, int);

        //implemented methods
        // base model (similarity transform)
        Eigen::MatrixXf buildA1Top();
        Eigen::MatrixXf buildA1Bottom();
        Eigen::VectorXf buildb1();

        //scale adjustement
        Eigen::MatrixXf buildA2Top();
        Eigen::MatrixXf buildA2Bottom();
        void buildb2(Eigen::VectorXf&, Eigen::VectorXf&);


        //
        void deformation_init();
        void scale_free_reconstruction();
        void scale_adjustement();
        void deform();

        //solving
        void construct_rhs();
        void construct_matrix();
        void construct_matrix_scale(); 

        //extensions
        void compute_barycenter(std::vector<Vec2>, std::vector<Vec2>);
        Eigen::MatrixXf buildA2Bottom_barycentric(std::vector<Vec2>, std::vector<Vec2>);
        void construct_matrix_scale_barycentric(); 

        void weighted_deform();
        //collision, depth adjustement

        //getters and setters
        Delaunay get_mesh() const {return _mesh;}; 
        std::vector<VertexType> get_fixed_vec() const {return _fixed_vec;};  
        std::vector<VertexType> get_handles_vec() const {return _handles_vec;};   
        std::vector<VertexType> get_handles_vec_deform() const {return _handles_vec_deform;};  

};

#endif DEFORMATION_H