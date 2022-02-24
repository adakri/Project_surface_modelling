/**
 * @file Mesh.cpp
 * @author A.Vivière, A.Dakri
 * @brief Class qui implémente une classe générique Mesh, et la classe Delaunay modifiée.
 * @version 0.1
 * @date 2022-01-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "Mesh.hpp"
#include "Deformation.hpp"

/**
 * @brief Construct a new Mesh:: Mesh object
 * Classe générique pour instancier les triangulations.
 * @param polygon
 */
Mesh::Mesh(Polygon *polygon) : _polygon(polygon)
{
}

/**
 * @brief Destroy the Mesh:: Mesh object
 * Destructeur par défaut.
 */
Mesh::~Mesh()
{
}

// Class delauny

/**
 * @brief Construct a new Delaunay:: Delaunay object
 * Classe qui implémente la triangulation de Delaunay modifiée par un algorithme Earcut.
 * Constructeur d'un polygone.
 * @param polygon
 */
Delaunay::Delaunay(Polygon *polygon) : Mesh(polygon)
{
	_resolution = 1;
	std::cout << "===========================================" << std::endl;
	std::cout << "The Construction of Delaunay object is done ! \n";

	this->triangulate_earclip();

	this->setupMesh();

	debug("exited delauney class")
}

/**
 * @brief Construct a new Delaunay:: Delaunay object
 * Constructeur à partir d'une mesh et d'une résolution donnée.
 * @param polygon
 * @param resolution
 */
Delaunay::Delaunay(Polygon *polygon, int resolution) : Mesh(polygon), _resolution(resolution)
{
	std::cout << "=====================================================" << std::endl;
	std::cout << "Meshing at resolution 0" << std::endl;
	this->triangulate_earclip();

	this->setupMesh();

	for (int i = 1; i <= _resolution; i++)
	{
		std::cout << "=====================================================" << std::endl;
		std::cout << "Meshing at resolution " << i << std::endl;
		this->update();
	}
	this->setupMesh();

	std::cout << "=====================================================" << std::endl;
	std::cout << "Meshing done" << std::endl;
	_mesh_graph.resize(_edges.size());
}

/**
 * @brief Destroy the Delaunay:: Delaunay object
 * Destructeur par défault.
 */
Delaunay::~Delaunay()
{
}

/**
 * @brief Sous conditions de validité, raffiner le maillage.
 * A chaque tour des triangles, si le triangle est "raffinable" supprimer de la liste des triangles et le remplacer par trois triangles définis par ses trois
 * points et le barycentre du triangle. Ajuster la list des edges et vertices accordement.
 */
void Delaunay::update()
{
	debug("updating") double old_mean = this->compute_mean_area();
	int N_old = _triangles.size();

	std::vector<TriangleType> triangle_tmp = _triangles;

	this->resetTriangles();
	this->resetEdges();
	// Use the clipping to construct new triangles
	// The vertices are updated with inner points

	// add points to poly
	int vtx_index(_vertices.size() - 1);

	for (std::size_t i = 0; i < N_old; i++)
	{
		triangle_tmp[i].check_validity();
		double area = triangle_tmp[i].compute_surface();

		bool isbad = triangle_tmp[i].get_is_Bad();

		if (!isbad)
		{
			// Updating the vertices
			VertexType vec = triangle_tmp[i].getCenter();
			vec._index = vtx_index + 1;
			_vertices.push_back(vec);

			// updating the edges and triangles
			VertexType tmp1, tmp2, tmp3, tmp4;
			tmp1 = triangle_tmp[i].get_a();
			tmp2 = triangle_tmp[i].get_b();
			tmp3 = triangle_tmp[i].get_c();
			tmp4 = VertexType(triangle_tmp[i].getCenter().getX(),
							  triangle_tmp[i].getCenter().getY());

			tmp4._index = vtx_index + 1;

			vtx_index++;

			_edges.push_back(EdgeType(tmp1, tmp2));
			_edges.push_back(EdgeType(tmp2, tmp3));
			_edges.push_back(EdgeType(tmp3, tmp1));
			_edges.push_back(EdgeType(tmp1, tmp4));
			_edges.push_back(EdgeType(tmp2, tmp4));
			_edges.push_back(EdgeType(tmp3, tmp4));

			TriangleType t1 = TriangleType(tmp1, tmp2, tmp4);
			TriangleType t2 = TriangleType(tmp1, tmp3, tmp4);
			TriangleType t3 = TriangleType(tmp2, tmp3, tmp4);

			t1.set_index(i * 3);
			t2.set_index(i * 3 + 1);
			t3.set_index(i * 3 + 2);

			_triangles.push_back(t1);
			_triangles.push_back(t2);
			_triangles.push_back(t3);
		}
		else
		{
			// updating the edges and triangles
			VertexType tmp1, tmp2, tmp3, tmp4;
			tmp1 = triangle_tmp[i].get_a();
			tmp2 = triangle_tmp[i].get_b();
			tmp3 = triangle_tmp[i].get_c();

			_edges.push_back(EdgeType(tmp1, tmp2));
			_edges.push_back(EdgeType(tmp2, tmp3));
			_edges.push_back(EdgeType(tmp3, tmp1));

			TriangleType t1 = TriangleType(tmp1, tmp2, tmp3);

			t1.set_index(triangle_tmp[i].get_index());

			_triangles.push_back(t1);
		}
	}
}

/**
 * @brief
 * Construit la hiérarchie et l'indexation de la triangulation.
 */
void Delaunay::setupMesh()
{
	debug("Inside set up mesh") 
	int t_count; // combien de triangle (si 1 bord, si 2 interieur )
	for (int i = 0; i < _edges.size(); i++)
	{
		t_count = 0;

		Edge e = _edges[i];

		std::vector<int> triangle_indices;

		for (int j = 0; j < _triangles.size(); j++)
		{
			Triangle t = _triangles[j];

			if (e == t.get_e1())
			{
				t_count++;
				triangle_indices.push_back(j);
			}
			else if (e == t.get_e2())
			{
				t_count++;
				triangle_indices.push_back(j);
			}
			else if (e == t.get_e3())
			{
				t_count++;
				triangle_indices.push_back(j);
			}
		}

		if (t_count == 1)
		{
			// boundary edges
			VertexType vi = e.get_v();
			VertexType vj = e.get_w();

			Triangle t1 = _triangles[triangle_indices[0]];

			VertexType vl = t1.third_vertex(vi, vj);

			std::array<Vec2, 3> neighbours = {vi, vj, vl};
			EdgeBoundary edge = EdgeBoundary(neighbours, e);

			int verdict(0);

			for (int i = 0; i < _edges_boundary.size(); i++)
			{

				if (_edges_boundary[i] == edge)
				{
					// this edge is repeated in edges inner
					verdict++;
					break;
				}
			}
			if (verdict == 0)
			{
				edge._index = i;
				_edges_boundary.push_back(edge);
			}
			else
			{
			}
		}
		else if (t_count == 2)
		{

			// is inner
			VertexType vi = _edges[i].get_v();
			VertexType vj = _edges[i].get_w();

			Triangle t1 = _triangles[triangle_indices[0]];
			Triangle t2 = _triangles[triangle_indices[1]];

			VertexType vl = t1.third_vertex(vi, vj);
			VertexType vr = t2.third_vertex(vi, vj);
			std::array<Vec2, 4> neighbours = {vi, vj, vl, vr};
			EdgeInner edge = EdgeInner(neighbours, e);
			int verdict(0);
			for (int i = 0; i < _edges_inner.size(); i++)
			{

				if (_edges_inner[i] == edge)
				{

					// this edge is repeated in edges inner
					verdict++;
					break;
				}
			}
			if (verdict == 0)
			{
				edge._index = i;
				_edges_inner.push_back(edge);
			}
			else
			{
				_edges.erase(_edges.begin() + i);
			}
		}
		else
		{
			std::cout << "A triangle is Bad, check debugg information with isBad" << std::endl;
			abort();
		}
	}
}

/**
 * @brief Implémente l'algorithme earclip.
 * 
 */
void Delaunay::triangulate_earclip()
{
	using Point = std::array<double, 2>;

	Polygon::print_polygon(_polygon->get_points());

	std::vector<Point> tmp;
	std::vector<std::vector<Point>> polyline;

	// debug(_polygon->get_points().size())

	int vtx_idx(0);

	for (int i = 0; i < _polygon->get_points().size(); i++)
	{
		tmp.push_back({_polygon->get_points()[i]._x,
					   _polygon->get_points()[i]._y});
		VertexType vec = _polygon->get_points()[i];
		vec._index = vtx_idx;
		_vertices.push_back(vec);
		vtx_idx++;
	}

	polyline.push_back(tmp);
	polyline.push_back({});

	std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(polyline);

	int i(0);


	while (i < indices.size())
	{
		// adding the edges and triangles (not the points)
		VertexType tmp1, tmp2, tmp3;
		tmp1 = _vertices[indices[i]];

		tmp2 = _vertices[indices[i + 1]];

		tmp3 = _vertices[indices[i + 2]];


		_edges.push_back(EdgeType(tmp1, tmp2));
		_edges.push_back(EdgeType(tmp2, tmp3));
		_edges.push_back(EdgeType(tmp3, tmp1));

		TriangleType t = TriangleType(tmp1, tmp2, tmp3);
		t.set_index(i);
		_triangles.push_back(t);
		i += 3;
	}
}

/**
 * @brief Effectur la triangulation de Delaunay modifiée.
 * 
 */
void Delaunay::triangulate()
{
	/* x0, y0, x1, y1, ... */
	std::vector<double> coords;

	for (int i = 0; i < _polygon->get_points().size(); i++)
	{
		std::cout << _polygon->get_points()[i] << std::endl;
		coords.push_back(_polygon->get_points()[i]._x);
		coords.push_back(_polygon->get_points()[i]._y);
	}


	for (auto e : coords)
		std::cout << e << std::endl;

	// triangulation happens here
	Delaunator d(coords);
	std::cout << d.triangles.size() << std::endl;

	int vtx_index(0.);
	for (std::size_t i = 0; i < d.triangles.size(); i += 3)
	{
		printf(
			"Triangle points: [[%f, %f], [%f, %f], [%f, %f]]\n",
			d.coords[2 * d.triangles[i]],		  // tx0
			d.coords[2 * d.triangles[i] + 1],	  // ty0
			d.coords[2 * d.triangles[i + 1]],	  // tx1
			d.coords[2 * d.triangles[i + 1] + 1], // ty1
			d.coords[2 * d.triangles[i + 2]],	  // tx2
			d.coords[2 * d.triangles[i + 2] + 1]  // ty2
		);

		VertexType tmp1, tmp2, tmp3;
		tmp1 = VertexType(d.coords[2 * d.triangles[i]], d.coords[2 * d.triangles[i] + 1]);
		tmp2 = VertexType(d.coords[2 * d.triangles[i + 1]], d.coords[2 * d.triangles[i + 1] + 1]);
		tmp3 = VertexType(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1]);


		_edges.push_back(EdgeType(tmp1, tmp2));
		_edges.push_back(EdgeType(tmp2, tmp3));
		_edges.push_back(EdgeType(tmp3, tmp1));

		TriangleType t = TriangleType(tmp1, tmp2, tmp3);
		t.set_index(i);
		_triangles.push_back(t);
	}
	std::cout << "===========================================" << std::endl;
	std::cout << "The Triangulation is done! \n";

}


/**
 * @brief Ajoute dans la structure des innerPoints.
 * 
 */
void Delaunay::createInnerPoints()
{
	for (int i = 0; i < _triangles.size(); i++)
	{
		_innerPoints.push_back(_triangles[i].getCenter());
	}
}

/**
 * @brief Reinitialise la liste des triangles.
 * 
 */
void Delaunay::resetTriangles()
{
	_triangles.clear();
}

/**
 * @brief Reinitialise la liste des aretes.
 * 
 */
void Delaunay::resetEdges()
{
	_edges.clear();
}

/**
 * @brief Reinitialise la liste des points.
 * 
 */
void Delaunay::resetVertices()
{
	_vertices.clear();
}


/**
 * @brief Surcharge de l'operateur d'affectation
 * 
 * @param m 
 * @return Delaunay& 
 */
Delaunay &Delaunay::operator=(const Delaunay &m)
{
	_triangles = m._triangles;
	_edges = m._edges;
	_innerPoints = m._innerPoints;
	_resolution = m._resolution;
	_mesh_graph = m._mesh_graph;
	_mesh_graph2 = m._mesh_graph2;
	_edges_boundary = m._edges_boundary;
	_edges_inner = m._edges_inner;
	_handlesIndex = m._handlesIndex;
	_vertices = m._vertices;
	return *this;
}

/**
 * @brief Dessine la triangulation à l'aide de SFML.
 * 
 * @param window Window that wraps all the visualisation.
 */
void Delaunay::draw_mesh(sf::RenderWindow &window)
{

	int N = _triangles.size();
	bool whileopen = true;
	// construct edges
	std::vector<std::array<sf::Vertex, 2>> line(N * 3);

	int i(0), j(0); // i on the triangles, j on the lines

	while (i < N)
	{
		line[j] = {
			sf::Vector2f(_triangles[i].get_a().getX(), _triangles[i].get_a().getY()),
			sf::Vector2f(_triangles[i].get_b().getX(), _triangles[i].get_b().getY())};
		line[j][0].color = sf::Color::Black;
		line[j][1].color = sf::Color::Black;
		j++;

		line[j] = {
			sf::Vector2f(_triangles[i].get_b().getX(), _triangles[i].get_b().getY()),
			sf::Vector2f(_triangles[i].get_c().getX(), _triangles[i].get_c().getY())};
		line[j][0].color = sf::Color::Black;
		line[j][1].color = sf::Color::Black;
		j++;

		line[j] = {
			sf::Vector2f(_triangles[i].get_c().getX(), _triangles[i].get_c().getY()),
			sf::Vector2f(_triangles[i].get_a().getX(), _triangles[i].get_a().getY())};
		line[j][0].color = sf::Color::Black;
		line[j][1].color = sf::Color::Black;
		j++;

		i++;
	}

	std::vector<sf::CircleShape> circs;
	for (int i = 0; i < _vertices.size(); i++)
	{
		sf::CircleShape circ(8);
		circ.setFillColor(sf::Color::Green);
		circ.setPosition(sf::Vector2f(_vertices[i].getX() - 8, _vertices[i].getY() - 8));
		circs.push_back(circ);
	}

	while (whileopen)
	{
		sf::Clock clock;
		window.clear(sf::Color::White);
		sf::Event event;

		for (int i = 0; i < 3 * N; i++)
		{
			sf::Vertex line_temp[] = {line[i][0], line[i][1]};
			window.draw(line_temp, 2, sf::Lines);
		}

		for (int i = 0; i < circs.size(); i++)
		{
			window.draw(circs[i]);
		}
		window.display();

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
		{
			window.close();
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
		{
			whileopen = false;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::C))
		{
			Polygon *polygon = new Polygon(*getPolygon());
		}
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)
			{
				std::cout << "click" << std::endl;

				int x = sf::Mouse::getPosition(window).x;
				int y = sf::Mouse::getPosition(window).y;
				std::cout << _vertices.size() << std::endl;
				for (int i = 0; i < _vertices.size(); i++)
				{
					int dx = x - _vertices[i].getX();
					int dy = y - _vertices[i].getY();
					float d = dx * dx + dy * dy;
					if (d <= 64)
					{
						std::cout << "ca marche" << std::endl;
						circs[i].setFillColor(sf::Color::Red);
						window.draw(circs[i]);
						if (std::find(_handlesIndex.begin(), _handlesIndex.end(), i) != _handlesIndex.end())
						{
							std::cout << "handle deja ajouté" << std::endl;
						}
						else
						{
							_handlesIndex.push_back(i);
						}
						std::cout << "***" << std::endl;
						for (int j = 0; j < _handlesIndex.size(); j++)
						{
						std:
							cout << _handlesIndex[j] << std::endl;
						}
					}
				}
			}
			if (event.type == sf::Event::Closed)
				window.close();
		}
	}
}

/**
 * @brief Constructs mesh graph, for the culture.
 * 
 */
void Delaunay::construct_mesh_graph()
{
	int t_count; // combien de triangle (si 1 bord, si 2 interieur )
	for (int i = 0; i < _edges.size(); i++)
	{
		t_count = 0;

		Edge e = _edges[i];

		for (int j = 0; j < _triangles.size(); j++)
		{
			Triangle t = _triangles[j];

			if (e == t.get_e1())
			{
				_mesh_graph[i].push_back(j);
				_mesh_graph[i].push_back(1);
				t_count++;
			}
			else if (e == t.get_e2())
			{
				_mesh_graph[i].push_back(j);
				_mesh_graph[i].push_back(2);
				t_count++;
			}
			else if (e == t.get_e3())
			{
				_mesh_graph[i].push_back(j);
				_mesh_graph[i].push_back(3);
				t_count++;
			}
		}

		if (t_count == 1)
		{
			_mesh_graph[i].push_back(0);
			_mesh_graph[i].push_back(0);
			// to normalize the size to N*8
			_mesh_graph[i].push_back(0); // is boundary
			// boundary edges
			VertexType vi = _edges[_mesh_graph[i][0]].get_v();
			VertexType vj = _edges[_mesh_graph[i][0]].get_w();

			Triangle t1 = _triangles[_mesh_graph[i][1]];
			Triangle t2 = _triangles[_mesh_graph[i][3]];
			VertexType vl = t1.third_vertex(vi, vj);
			VertexType vr = t2.third_vertex(vi, vj);

		}
		else if (t_count == 2)
		{
			_mesh_graph[i].push_back(1); // is inner
		}
		else
		{
			std::cout << "A triangle is Bad, check debugg information fwith isBad" << std::endl;
			abort();
		}
	}

	// update the edge
}

/**
 * @brief Affiche le graphe de la triangulation.
 * 
 */
void Delaunay::print_mesh_graph()
{
	std::cout << "===============================================================" << std::endl;
	std::cout << "The mesh graph" << std::endl;
	printf("%-10s%-13s%-10s%-13s%-10s%-10s \n",
		   "edge i",
		   "triangle1 i", "edge i",
		   "triangle2 i", "edge i",
		   "is it an inner edge");
	for (int i = 0; i < _mesh_graph.size(); i++)
	{
		printf("%-12d%-12d%-12d%-12d%-12d%-12d \n",
			   i,
			   _mesh_graph[i][0], _mesh_graph[i][1],
			   _mesh_graph[i][2], _mesh_graph[i][3],
			   _mesh_graph[i][4]);
	}
	std::cout << "===============================================================" << std::endl;
}

/**
 * @brief Met à jour les points.
 * 
 */
void Delaunay::refresh_with_vertices()
{
	for (int i = 0; i < _edges.size(); i++)
	{
		int v_idx = _edges[i].get_v()._index;
		int w_idx = _edges[i].get_w()._index;
		_edges[i] = EdgeType(_vertices[v_idx], _vertices[w_idx]);
	}

	for (int i = 0; i < _triangles.size(); i++)
	{
		int a_idx = _triangles[i].get_a()._index;
		int b_idx = _triangles[i].get_b()._index;
		int c_idx = _triangles[i].get_c()._index;

		_triangles[i] = TriangleType(_vertices[a_idx], _vertices[b_idx], _vertices[c_idx]);
	}
	std::cout << "============================UPDATED THE MESH===================================" << std::endl;
}

/**
 * @brief Calcule la surface moyenne des triangles dans la triangulation.
 * 
 * @return double 
 */
double Delaunay::compute_mean_area()
{
	double mean(0.);
	for (auto t : _triangles)
	{
		mean += t.compute_surface();
	}
	return mean / _triangles.size();
}
