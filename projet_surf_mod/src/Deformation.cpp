/**
 * @file Deformation.cpp
 * @author A.Vivière, A.Dakri
 * @brief
 * @version 0.1
 * @date 2022-01-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "Deformation.hpp"

/**
 * @brief Construct a new Deformation:: Deformation object
 * Constructeur de la class déformation à partir d'une polyligne, de poignets et de points fixes (résolution 0 par défaut).
 * @param polygon
 * @param mesh
 * @param handles
 * @param fixed_vertices
 */
Deformation::Deformation(Polygon &polygon, Delaunay &mesh, std::vector<HANDLE> handles, std::vector<int> fixed_vertices) : _polygon(polygon), _handles(handles), _fixed_vertices(fixed_vertices)
{
    //_mesh = mesh;
    _N_fix = _fixed_vertices.size();
    _N_meshp = _mesh.getVertices().size();
    _N_handles = _handles.size();
}

/**
 * @brief Construct a new Deformation:: Deformation object
 * Constructeur de la class déformation à partir d'une polyligne, de poignets, de points fixes et de résolution.
 * @param polygon
 * @param handles
 * @param fixed_vertices
 * @param resolution
 */
Deformation::Deformation(Polygon &polygon, std::vector<HANDLE> handles, std::vector<int> fixed_vertices, int resolution) : _polygon(polygon), _handles(handles), _fixed_vertices(fixed_vertices), _resolution(resolution)
{
    _mesh = Delaunay(&polygon, _resolution);
    _N_fix = _fixed_vertices.size();
    _N_meshp = _mesh.getVertices().size();
    _N_handles = _handles.size();

    _fixed_vec_bary.resize(_N_handles);
    _handles_vec_bary.resize(_N_fix);

    for (auto ek : _mesh.get_edges_inner())
    {
        VertexType vi = ek._neighbours[0];
        VertexType vj = ek._neighbours[1];
        VertexType vl = ek._neighbours[2];
        VertexType vr = ek._neighbours[3];
    }

    for (auto e : _handles)
    {
        _handles_vec.push_back(_mesh.getVertices()[e.hand_index]);
        _handles_vec_deform.push_back(Vec2(e.deformation_en_x, e.deformation_en_y));
    }
    for (auto e : _fixed_vertices)
    {
        _fixed_vec.push_back(_mesh.getVertices()[e]);
    }
}

/**
 * @brief Initialise la déformation.
 *
 */

void Deformation::deformation_init()
{

    if (_N_meshp - _N_handles - _N_fix < 0)
    {
        std::cout << "===========================================" << std::endl;
        std::cout << "The number of constarints is bigger than the other points! \n";
        abort();
    }
    else
    {
        _points.resize(_N_meshp - _N_handles - _N_fix);
        // constructing points from the _vertices of mesh to points in eigen
        std::vector<Vec2> local_original_vertices = _mesh.getVertices();

        for (int i = 0; i < _N_handles; i++)
        {
            local_original_vertices.erase(local_original_vertices.begin() + _handles[i].hand_index);
        }
        for (int i = 0; i < _N_fix; i++)
        {
            local_original_vertices.erase(local_original_vertices.begin() + _fixed_vertices[i]);
        }
        // the local is updated
        for (int i = 0; i < _points.size(); i++)
        {
            _points[i] = local_original_vertices[i];
            _points[i].isString();
        }
    }
}

/**
 * @brief Construit le membre de droite.
 *
 */
void Deformation::construct_rhs()
{
    Eigen::VectorXf rhs(_N_meshp);
    for (int i = 0; i < _N_meshp; i++)
    {
        if (i < _N_meshp - _N_handles - _N_fix)
        {
            rhs[i] = 0.;
        }
        else if (i < _N_meshp - _N_handles)
        {
            rhs[i] = 40.; // handles
        }
        else
        {
            rhs[i] = 0.; // fixes
        }
    }
}

/**
 * @brief Construit la partie sup de A1.
 * 
 * @return Eigen::MatrixXf 
 */
Eigen::MatrixXf Deformation::buildA1Top()
{
    Eigen::MatrixXf L1 = Eigen::MatrixXf::Zero(2 * (_mesh.get_edges_inner().size() + _mesh.get_edges_boundary().size()), 2 * _mesh.getVertices().size());


    Eigen::MatrixXf Gk;
    int k = 0;

    //_mesh.print_mesh_edges_inner();
    for (auto &ek : _mesh.get_edges_inner())
    {
        VertexType vi = ek._neighbours[0];
        VertexType vj = ek._neighbours[1];
        VertexType vl = ek._neighbours[2];
        VertexType vr = ek._neighbours[3];


        Vec2 ek_v = ek._edge_vector.get_reduced_vector(); // reduced vector

        Eigen::VectorXf ek_eigen(2);


        ek_eigen << ek_v.getX(), ek_v.getY();

        Eigen::Matrix2f ek_mat;
        ek_mat << ek_eigen[0], ek_eigen[1],
                  ek_eigen[1], -ek_eigen[0];

        float vix = vi.getX();
        float viy = vi.getY();
        float vjx = vj.getX();
        float vjy = vj.getY();
        float vlx = vl.getX();
        float vly = vl.getY();
        float vrx = vr.getX();
        float vry = vr.getY();
        Gk = (Eigen::MatrixXf(8, 2) << vix, viy,
              viy, -vix,
              vjx, vjy,
              vjy, -vjx,
              vlx, vly,
              vly, -vlx,
              vrx, vry,
              vry, -vrx)
                 .finished();

        
        std::cout<< ek_mat << std::endl;

        Eigen::MatrixXf hk = constantM_inner - ek_mat * (Gk.transpose() * Gk).inverse() * Gk.transpose();
        std::cout<< hk << std::endl;

        int vix_index = 2 * vi.get_index();
        int viy_index = 2 * vi.get_index() + 1;
        int vjx_index = 2 * vj.get_index();
        int vjy_index = 2 * vj.get_index() + 1;
        int vlx_index = 2 * vl.get_index();
        int vly_index = 2 * vl.get_index() + 1;
        int vrx_index = 2 * vr.get_index();
        int vry_index = 2 * vr.get_index() + 1;
        int ekx_index = 2 * k;
        int eky_index = 2 * k + 1;



        L1(ekx_index, vix_index) = hk(0, 0);
        L1(eky_index, vix_index) = hk(1, 0);
        L1(ekx_index, viy_index) = hk(0, 1);
        L1(eky_index, viy_index) = hk(1, 1);
        L1(ekx_index, vjx_index) = hk(0, 2);
        L1(eky_index, vjx_index) = hk(1, 2);
        L1(ekx_index, vjy_index) = hk(0, 3);
        L1(eky_index, vjy_index) = hk(1, 3);
        L1(ekx_index, vlx_index) = hk(0, 4);
        L1(eky_index, vlx_index) = hk(1, 4);
        L1(ekx_index, vly_index) = hk(0, 5);
        L1(eky_index, vly_index) = hk(1, 5);
        L1(ekx_index, vrx_index) = hk(0, 6);
        L1(eky_index, vrx_index) = hk(1, 6);
        L1(ekx_index, vry_index) = hk(0, 7);
        L1(eky_index, vry_index) = hk(1, 7);

        k++;
    }

    for(auto& ek : _mesh.get_edges_boundary())
    {

        ek.print_edge();


        VertexType vi = ek._neighbours[0];
        VertexType vj = ek._neighbours[1];
        VertexType vl = ek._neighbours[2];

        Vec2 ek_v = ek._edge_vector.get_reduced_vector();

        Eigen::VectorXf ek_eigen(2);
        ek_eigen << ek_v.getX(), ek_v.getY();

        Eigen::Matrix2f ek_mat;
        ek_mat << ek_eigen[0], ek_eigen[1],
            ek_eigen[1], -ek_eigen[0];

        float vix = vi.getX();
        float viy = vi.getY();
        float vjx = vj.getX();
        float vjy = vj.getY();
        float vlx = vl.getX();
        float vly = vl.getY();

        std::cout << ek_mat << std::endl;

        Gk = (Eigen::MatrixXf(6, 2) << vix, viy,
              viy, -vix,
              vjx, vjy,
              vjy, -vjx,
              vlx, vly,
              vly, -vlx)
                 .finished();


        Eigen::MatrixXf hk = constantM_boundary - ek_mat * (Gk.transpose() * Gk).inverse() * Gk.transpose();

        int vix_index = 2 * vi.get_index();
        int viy_index = 2 * vi.get_index() + 1;
        int vjx_index = 2 * vj.get_index();
        int vjy_index = 2 * vj.get_index() + 1;
        int vlx_index = 2 * vl.get_index();
        int vly_index = 2 * vl.get_index() + 1;
        int ekx_index = 2 * k;
        int eky_index = 2 * k + 1;

        L1(ekx_index, vix_index) = hk(0, 0);
        L1(eky_index, vix_index) = hk(1, 0);
        L1(ekx_index, viy_index) = hk(0, 1);
        L1(eky_index, viy_index) = hk(1, 1);
        L1(ekx_index, vjx_index) = hk(0, 2);
        L1(eky_index, vjx_index) = hk(1, 2);
        L1(ekx_index, vjy_index) = hk(0, 3);
        L1(eky_index, vjy_index) = hk(1, 3);
        L1(ekx_index, vlx_index) = hk(0, 4);
        L1(eky_index, vlx_index) = hk(1, 4);
        L1(ekx_index, vly_index) = hk(0, 5);
        L1(eky_index, vly_index) = hk(1, 5);

        k++;
    }
    std::cout<<"===========================L1============================"<<std::endl;
    std::cout<< L1 << std::endl;

    return L1;
}

/**
 * @brief Construit le bas de A2.
 * 
 * @return Eigen::MatrixXf 
 */
Eigen::MatrixXf Deformation::buildA1Bottom()
{
    int n_control_points = _N_handles + _N_fix;
    Eigen::MatrixXf C1 = Eigen::MatrixXf::Zero(2 * n_control_points, 2 * _mesh.getVertices().size());


    for (int ci = 0; ci < _N_fix; ++ci)
    {
        int cix_index = 2 * ci;
        int ciy_index = 2 * ci + 1;

        this->get_fixed_vec()[ci].isString();
        int w0_vertex_id = this->get_fixed_vec()[ci]._index;



        C1(cix_index, 2 * w0_vertex_id) = w;
        C1(ciy_index, 2 * w0_vertex_id + 1) = w;
    }


    int incrementor(0);
    for (int ci = _N_fix; ci < n_control_points; ++ci)
    {
        int cix_index = 2 * ci;
        int ciy_index = 2 * ci + 1;


        this->get_handles_vec()[incrementor].isString();
        
        int w0_vertex_id = this->get_handles_vec()[incrementor]._index;


        C1(cix_index, 2 * w0_vertex_id) = w;
        C1(ciy_index, 2 * w0_vertex_id + 1) = w;

        incrementor++;
    }

    std::cout<<"===========================C1========================"<<std::endl;
    std::cout<< C1 << std::endl;

    return C1;
}

/**
 * @brief Construit le second terme du prmier modèle.
 * 
 * @return Eigen::VectorXf 
 */
Eigen::VectorXf Deformation::buildb1()
{
    Eigen::VectorXf b1 = Eigen::VectorXf::Zero(2 * (_mesh.get_edges_inner().size() + _mesh.get_edges_boundary().size()) + 2 * (_N_fix + _N_handles));

    int edge_vectors_size = 2 * (_mesh.get_edges_inner().size() + _mesh.get_edges_boundary().size());

        for (int ci = 0; ci < _N_fix; ++ci)
    {
        int cix_index = edge_vectors_size + 2 * ci;
        int ciy_index = edge_vectors_size + 2 * ci + 1;

        float cix = this->get_fixed_vec()[ci].getX();
        float ciy = this->get_fixed_vec()[ci].getY();

        b1(cix_index) = w * cix;
        b1(ciy_index) = w * ciy;
    }

    for (int ci = 0; ci < _N_handles; ++ci)
    {
        int cix_index = edge_vectors_size + 2 * _N_fix + 2 * ci;
        int ciy_index = edge_vectors_size + 2 * _N_fix + 2 * ci + 1;

        float cix = this->get_handles_vec()[ci].getX();
        float ciy = this->get_handles_vec()[ci].getY();
        float cixd = this->get_handles_vec_deform()[ci].getX();
        float ciyd = this->get_handles_vec_deform()[ci].getY();

        b1(cix_index) = w * (cixd);
        b1(ciy_index) = w * (ciyd);
    }

    std::cout<<"=====================================b1================================"<<std::endl;

    return b1;
}


/**
 * @brief Construit et r"sout le système.
 * 
 */
void Deformation::construct_matrix()
{
    int n_control_points = _N_handles + _N_fix;

    Eigen::MatrixXf A1(2 * (_mesh.get_edges_inner().size() + _mesh.get_edges_boundary().size()) + 2 * n_control_points, 2 * _mesh.getVertices().size());

    Eigen::VectorXf b1 = Eigen::VectorXf::Zero(2 * (_mesh.get_edges_boundary().size() + _mesh.get_edges_inner().size()) + 2 * (_N_fix + _N_handles));

    A1.topRightCorner(2 * (_mesh.get_edges_inner().size() + _mesh.get_edges_boundary().size()), 2 * _mesh.getVertices().size()) = this->buildA1Top();
    A1.bottomLeftCorner(2 * n_control_points, 2 * _mesh.getVertices().size()) = this->buildA1Bottom();

    b1 = buildb1();
    // solving using normal equations

    Eigen::VectorXf v = ((A1.transpose() * A1)).ldlt().solve(A1.transpose() * b1);



    for (int i = 0; i < _mesh.getVertices().size(); ++i)
    {
        float vx, vy;
        vx = v[2 * i];
        vy = v[2 * i + 1];

        VertexType tmp = VertexType(vx, vy);
        tmp._index = _mesh._vertices[i]._index;
        _mesh._vertices[tmp._index] = tmp;
    }

    Delaunay old = _mesh;
    _mesh.refresh_with_vertices();

    std::cout<<"========================================================="<<std::endl;
    std::cout<<"===============Finished the calcul======================="<<std::endl;
    std::cout<<"========================================================="<<std::endl;

}

/**
 * @brief Construire la partie sup du deuxième modèle.
 * 
 * @return Eigen::MatrixXf 
 */
Eigen::MatrixXf Deformation::buildA2Top()
{
    // meme principe
    Eigen::MatrixXf L2 = Eigen::MatrixXf::Zero(_mesh.get_edges_inner().size() + _mesh.get_edges_boundary().size(), _mesh.getVertices().size());

    int k = 0;

    for (const auto &ek : _mesh.get_edges_inner())
    {

        VertexType vi = ek._neighbours[0];
        VertexType vj = ek._neighbours[1];

        int vi_index = vi._index;
        int vj_index = vj._index;

        int ek_index = k;

        L2(ek_index, vi_index) = -1;
        L2(ek_index, vj_index) = 1;

        k++;
    }

    k = _mesh.get_edges_inner().size();

    for (const auto &ek : _mesh.get_edges_boundary())
    {
        VertexType vi = ek._neighbours[0];
        VertexType vj = ek._neighbours[1];

        int vi_index = vi._index;
        int vj_index = vj._index;

        int ek_index = k;

        L2(ek_index, vi_index) = -1;
        L2(ek_index, vj_index) = 1;

        k++;
    }
    std::cout<<"==============================L2============================"<<std::endl;
    std::cout<< L2 << std::endl;

    return L2;
}

/**
 * @brief Construit la partie inf de la matr
 * 
 * @return Eigen::MatrixXf 
 */
Eigen::MatrixXf Deformation::buildA2Bottom()
{
    int n_control_points = _N_handles + _N_fix;

    Eigen::MatrixXf C2 = Eigen::MatrixXf::Zero(n_control_points, _mesh.getVertices().size());


    for (int ci = 0; ci < _N_fix; ++ci)
    {
        int cix_index = ci;

        int w0_vertex_id = this->get_fixed_vec()[ci]._index;

        C2(cix_index, w0_vertex_id) = w;
    }


    int incrementor(0);
    
    for (int ci = _N_fix; ci < n_control_points; ++ci)
    {
        int cix_index = ci;
        int ciy_index = 2 * ci + 1;



        debug(this->get_handles_vec()[incrementor]._index)

        int w0_vertex_id = this->get_handles_vec()[incrementor]._index;

        C2(cix_index, w0_vertex_id) = w;

        incrementor++;
    }

    std::cout<<"==============================C2============================"<<std::endl;
    std::cout<< C2 << std::endl;

    return C2;
}

/**
 * @brief Construit le second terme du système du deuxième modèle.
 * 
 * @param b2x 
 * @param b2y 
 */
void Deformation::buildb2(Eigen::VectorXf &b2x, Eigen::VectorXf &b2y)
{
    int n_control_points = _N_handles + _N_fix;


    int k = 0;


    for (const auto &ek : _mesh.get_edges_inner())
    {
        VertexType vi = ek._neighbours[0];
        VertexType vj = ek._neighbours[1];
        VertexType vl = ek._neighbours[2];
        VertexType vr = ek._neighbours[3];

        float vix = vi.getX();
        float viy = vi.getY();
        float vjx = vj.getX();
        float vjy = vj.getY();
        float vlx = vl.getX();
        float vly = vl.getY();
        float vrx = vr.getX();
        float vry = vr.getY();

        Eigen::MatrixXf Gk = (Eigen::MatrixXf(8, 2) << vix, viy,
                              viy, -vix,
                              vjx, vjy,
                              vjy, -vjx,
                              vlx, vly,
                              vly, -vlx,
                              vrx, vry,
                              vry, -vrx)
                                 .finished();



        Eigen::VectorXf vek(8);
        vek<< vix,viy, vjx, vjy,
              vlx, vly, vrx, vry;


        Eigen::VectorXf cksk = (Gk.transpose() * Gk).inverse() * Gk.transpose() * vek;

        std::cout << cksk << std::endl;

        float ck = cksk(0);
        float sk = cksk(1);
        Eigen::Matrix2f Tk;
        Tk << ck, sk,
             -sk, ck;


        Tk = Tk * (1.0f / sqrt((pow(ck, 2) + pow(sk, 2))));

        Vec2 ek_v = ek._edge_vector.get_reduced_vector();


        Eigen::VectorXf ek_eigen(2);

        ek_eigen << ek_v.getX(), ek_v.getY();

        ek_eigen = Tk * ek_eigen;


        b2x(k) = ek_eigen(0);
        b2y(k) = ek_eigen(1);


        k++;
    }

    k = _mesh.get_edges_inner().size();


    for (const auto &ek : _mesh.get_edges_boundary())
    {
        VertexType vi = ek._neighbours[0];
        VertexType vj = ek._neighbours[1];
        VertexType vl = ek._neighbours[2];

        Vec2 ek_v = ek._edge_vector.get_reduced_vector();

        Eigen::VectorXf ek_eigen(2);
        ek_eigen << ek_v.getX(), ek_v.getY();

        float vix = vi.getX();
        float viy = vi.getY();
        float vjx = vj.getX();
        float vjy = vj.getY();
        float vlx = vl.getX();
        float vly = vl.getY();

        Eigen::MatrixXf Gk = (Eigen::MatrixXf(6, 2) << vix, viy,
                              viy, -vix,
                              vjx, vjy,
                              vjy, -vjx,
                              vlx, vly,
                              vly, -vlx)
                                 .finished();

        Eigen::VectorXf vek(6, 1);
        vek << vix, viy, vjx, vjy, vlx, vly;

        Eigen::Vector2f cksk = (Gk.transpose() * Gk).inverse() * Gk.transpose() * vek;

        float ck = cksk(0);
        float sk = cksk(1);
        Eigen::Matrix2f Tk;
        Tk << ck, sk,
            -sk, ck;

        Tk = Tk * (1.0f / sqrt((pow(ck, 2) + pow(sk, 2))));

        ek_eigen = Tk * ek_eigen;

        b2x(k) = ek_eigen(0);
        b2y(k) = ek_eigen(1);

        k++;
    }

    int edge_vectors_size = _mesh.get_edges_boundary().size() + _mesh.get_edges_inner().size();


    for (int ci = 0; ci < _N_fix; ++ci)
    {
        int cix_index = edge_vectors_size + ci;
        int ciy_index = edge_vectors_size + ci;

        float cix = this->get_fixed_vec()[ci].getX();
        float ciy = this->get_fixed_vec()[ci].getY();

        b2x(cix_index) = w * cix;

        b2y(ciy_index) = w * ciy;
    }


    for (int ci = 0; ci < _N_handles; ++ci)
    {
        int cix_index = edge_vectors_size + _N_fix + ci;
        int ciy_index = edge_vectors_size + _N_fix + ci;

        float cix = this->get_handles_vec()[ci].getX();
        float ciy = this->get_handles_vec()[ci].getY();
        float cixd = this->get_handles_vec_deform()[ci].getX();
        float ciyd = this->get_handles_vec_deform()[ci].getY();

        b2x(cix_index) = w * (cixd);

        b2y(ciy_index) = w * (ciyd);
    }
}

/**
 * @brief Construit et résout le système du deuxième modèle.
 * 
 */
void Deformation::construct_matrix_scale()
{
    int n_control_points = _N_handles + _N_fix;
    Eigen::VectorXf b2x = Eigen::VectorXf::Zero(_mesh.get_edges_boundary().size() + _mesh.get_edges_inner().size() + n_control_points);

    Eigen::VectorXf b2y = Eigen::VectorXf::Zero(_mesh.get_edges_boundary().size() + _mesh.get_edges_inner().size() + n_control_points);

    Eigen::MatrixXf A2(_mesh.get_edges_inner().size() + _mesh.get_edges_boundary().size() + n_control_points, _mesh.getVertices().size());

    this->construct_matrix();
    

    A2.topRightCorner(_mesh.get_edges_inner().size() + _mesh.get_edges_boundary().size(), _mesh.getVertices().size()) = this->buildA2Top();
    
    A2.bottomLeftCorner(n_control_points, _mesh.getVertices().size()) = this->buildA2Bottom();


    this->buildb2(b2x, b2y);

    // solving using normal equations twice
        
    Eigen::VectorXf vx = ((A2.transpose() * A2)).ldlt().solve(A2.transpose() * b2x);

    Eigen::VectorXf vy = ((A2.transpose() * A2)).ldlt().solve(A2.transpose() * b2y);

    for (int i = 0; i < _mesh.getVertices().size(); ++i)
    {
        float vx_f, vy_f;
        
        vx_f = vx[i];
        vy_f = vy[i];

        VertexType tmp = VertexType(vx_f, vy_f);
        
        tmp._index = _mesh._vertices[i]._index;
        
        _mesh._vertices[tmp._index] = tmp;
    }
    std::cout<<"========================================================="<<std::endl;
    std::cout<<"===============Finished the calcul======================="<<std::endl;
    std::cout<<"========================================================="<<std::endl;

    Delaunay old = _mesh;
    // updating the mesh
    _mesh.refresh_with_vertices();
}

/**
 * @brief Calcule les barycentres pour la déformation à main levée.
 * 
 * @param fix_b 
 * @param handles_b 
 */
void Deformation::compute_barycenter(std::vector<Vec2> fix_b, std::vector<Vec2> handles_b)
{
    int iterator(0);
    
    for (int ci = 0; ci < _N_handles; ++ci)
    {
        // this for the actual implementation
        Vec2 p = handles_b[ci];

        p.isString();
        // checking which triangle contains the vertex and compute barycentric
        
        iterator = 0;
        for (auto t : _mesh.getTriangles())
        {
            if (t.contains_point(p))
            {
                t.print_triangle();
                iterator++;
                float u, v, w;
                t.Barycentric(p, u, v, w);
                _handles_vec_bary[ci] = {u, v, w, float(t.get_index())};
            }
        }
    }

    for (int ci = 0; ci < _N_fix; ++ci)
    {

        Vec2 p = fix_b[ci];
        
        p.isString();

        // checking which triangle contains the vertex and compute barycentric
        iterator = 0;
        for (auto t : _mesh.getTriangles())
        {
            if (t.contains_point(p))
            {
                iterator++;
                float u, v, w;
                t.Barycentric(p, u, v, w);
                _fixed_vec_bary[ci] = {u, v, w, float(t.get_index())};
            }
        }
    }
}

Eigen::MatrixXf Deformation::buildA2Bottom_barycentric(std::vector<Vec2> fix, std::vector<Vec2> handles)
{
    this->compute_barycenter(fix, handles);
    
    int n_control_points = _N_handles + _N_fix;

    Eigen::MatrixXf C3 = Eigen::MatrixXf::Zero(n_control_points, _mesh.getVertices().size());


    for (int ci = 0; ci < _N_fix; ++ci)
    {
        this->get_fixed_vec()[ci].isString();

        int triangle_id(_handles_vec_bary[ci][3]);

        float w0 = _handles_vec_bary[ci][0];
        int w0_vertex_id = _mesh.getTriangles()[triangle_id].get_a()._index;
        C3(ci, w0_vertex_id) = w * w0;

        float w1 = _handles_vec_bary[ci][1];
        int w1_vertex_id = _mesh.getTriangles()[triangle_id].get_b()._index;
        C3(ci, w1_vertex_id) = w * w1;

        float w2 = _handles_vec_bary[ci][2];
        int w2_vertex_id = _mesh.getTriangles()[triangle_id].get_c()._index;
        C3(ci, w2_vertex_id) = w * w2;
    }


    int incrementor(0);
    
    for (int ci = _N_fix; ci < n_control_points; ++ci)
    {
        this->get_handles_vec()[incrementor].isString();

        int triangle_id(_fixed_vec_bary[incrementor][3]);

        float w0 = _fixed_vec_bary[incrementor][0];
        int w0_vertex_id = _mesh.getTriangles()[triangle_id].get_a()._index;
        C3(ci, w0_vertex_id) = w * w0;

        float w1 = _fixed_vec_bary[incrementor][1];
        int w1_vertex_id = _mesh.getTriangles()[triangle_id].get_b()._index;
        C3(ci, w1_vertex_id) = w * w1;

        float w2 = _fixed_vec_bary[incrementor][2];
        int w2_vertex_id = _mesh.getTriangles()[triangle_id].get_c()._index;
        C3(ci, w2_vertex_id) = w * w2;

        incrementor++;
    }



    std::cout<< C3 << std::endl;

    return C3;
}

void Deformation::construct_matrix_scale_barycentric()
{
    int n_control_points = _N_handles + _N_fix;
    Eigen::VectorXf b2x = Eigen::VectorXf::Zero(_mesh.get_edges_boundary().size() + _mesh.get_edges_inner().size() + n_control_points);

    Eigen::VectorXf b2y = Eigen::VectorXf::Zero(_mesh.get_edges_boundary().size() + _mesh.get_edges_inner().size() + n_control_points);

    Eigen::MatrixXf A2(_mesh.get_edges_inner().size() + _mesh.get_edges_boundary().size() + n_control_points, _mesh.getVertices().size());

    this->construct_matrix();
    

    A2.topRightCorner(_mesh.get_edges_inner().size() + _mesh.get_edges_boundary().size(), _mesh.getVertices().size()) = this->buildA2Top();
    
    this->buildb2(b2x, b2y);

    Eigen::VectorXf vx = ((A2.transpose() * A2)).ldlt().solve(A2.transpose() * b2x);
    
    Eigen::VectorXf vy = ((A2.transpose() * A2)).ldlt().solve(A2.transpose() * b2y);

    for (int i = 0; i < _mesh.getVertices().size(); ++i)
    {
        float vx_f, vy_f;
        vx_f = vx[i];
        vy_f = vy[i];

        VertexType tmp = VertexType(vx_f, vy_f);
        tmp._index = _mesh._vertices[i]._index;
        _mesh._vertices[tmp._index] = tmp;
    }
    std::cout<<"========================================================="<<std::endl;
    std::cout<<"===============Finished the calcul======================="<<std::endl;
    std::cout<<"========================================================="<<std::endl;

    Delaunay old = _mesh;
    // updating the mesh
    _mesh.refresh_with_vertices();
}
