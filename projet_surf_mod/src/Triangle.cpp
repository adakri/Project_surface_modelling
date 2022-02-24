/**
 * @file Triangle.cpp
 * @author A.Vivière & A.Dakri
 * @brief Classe qui implémente un triangle.
 * @version 0.1
 * @date 2022-01-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "Triangle.hpp"
#include <algorithm>
#include <list>

/**
 * @brief Construct a new Triangle:: Triangle object
 * Constructeur de la classe.
 * @param v1
 * @param v2
 * @param v3
 */
Triangle::Triangle(const VertexType v1, const VertexType v2, const VertexType v3) : _a(v1), _b(v2), _c(v3), _isBad(false)
{
}

/**
 * @brief
 * Egalité des triangles dans le sens des aretes, dans n'inporte quel ordre.
 * @param t1
 * @param t2
 * @return true
 * @return false
 */
bool almost_equal(const Triangle &t1, const Triangle &t2)
{
    return (VertexType::almost_equal(t1.get_a(), t2.get_a()) || VertexType::almost_equal(t1.get_a(), t2.get_b()) || VertexType::almost_equal(t1.get_a(), t2.get_c())) &&
           (VertexType::almost_equal(t1.get_b(), t2.get_a()) || VertexType::almost_equal(t1.get_b(), t2.get_b()) || VertexType::almost_equal(t1.get_b(), t2.get_c())) &&
           (VertexType::almost_equal(t1.get_c(), t2.get_a()) || VertexType::almost_equal(t1.get_c(), t2.get_b()) || VertexType::almost_equal(t1.get_c(), t2.get_c()));
}

/**
 * @brief Qualifie si un point est contenu dans un triangle.
 *
 * @param v
 * @return true
 * @return false
 */
bool Triangle::containsVertex(const VertexType &v)
{
    return VertexType::almost_equal(_a, v) || VertexType::almost_equal(_b, v) || VertexType::almost_equal(_c, v);
}

/**
 * @brief Trouve le centre(barycentre) d'un triangle.
 *
 * @return VertexType
 */
VertexType Triangle::getCenter()
{
    double nx = (get_a().getX() + get_b().getX() + get_c().getX()) / 3;
    double ny = (get_a().getY() + get_b().getY() + get_c().getY()) / 3;
    return VertexType(nx, ny);
}

/**
 * @brief Trouve le centre du cercle circonscrit dans un triangle.
 *
 */
void Triangle::findCircumCenter()
{
    // triangle PQR
    //  Line PQ is represented as ax + by = c
    spec_t a, b, c;
    _e1.lineFromVexrtex(a, b, c);

    // Line QR is represented as ex + fy = g
    spec_t e, f, g;
    _e2.lineFromVexrtex(e, f, g);

    // Converting lines PQ and QR to perpendicular
    // vbisectors. After this, L = ax + by = c
    // M = ex + fy = g
    _e1.perpendicularBisectorFromLine(a, b, c);
    _e2.perpendicularBisectorFromLine(e, f, g);

    // The point of intersection of L and M gives
    // the circumcenter
    _CircumCenter = Edge::lineLineIntersection(a, b, c, e, f, g);

    if (_CircumCenter._x == std::numeric_limits<spec_t>::max() &&
        _CircumCenter._y == std::numeric_limits<spec_t>::max())
    {
        std::cout << "The two perpendicular bisectors found parallel" << std::endl;
        std::cout << "Thus, the given points do not form a triangle and are collinear" << std::endl;
        _CircumRadius = std::numeric_limits<spec_t>::max();
    }

    else
    {
        std::cout << "The circumcenter of the triangle is: ";
        std::cout << "(" << _CircumCenter._x << ", " << _CircumCenter._y << ")" << std::endl;
        _CircumRadius = (_CircumCenter - _a).norm2();
    }
}

/**
 * @brief Trouve si le cercle circonscrit du triangle contient un point.
 *
 * @param v
 * @return true
 * @return false
 */
bool Triangle::Cercle_Contains(const VertexType &v) const
{
    return (v.dist(_CircumCenter) <= _CircumRadius);
}

/**
 * @brief Surcharge de l'opérateur affectation.
 *
 * @param t
 * @return Triangle&
 */
Triangle &Triangle::operator=(const Triangle &t)
{
    _a = t._a;
    _b = t._b;
    _c = t._c;
    // could not const it or size it ?
    _e1 = t._e1;
    _e2 = t._e2;
    _e3 = t._e3;
    _CircumCenter = t._CircumCenter;
    _CircumRadius = t._CircumRadius;
    _isBad = t._isBad;
    _index = t._index;
    return *this;
}

/**
 * @brief Surcharge de l'opérateur égalité.
 *
 * @param t
 * @return true
 * @return false
 */

bool Triangle::operator==(const Triangle &t) const
{
    return (this->_a == t._a || this->_a == t._b || this->_a == t._c) &&
           (this->_b == t._a || this->_b == t._b || this->_b == t._c) &&
           (this->_c == t._a || this->_c == t._b || this->_c == t._c);
}

std::ostream &operator<<(std::ostream &str, const Triangle &t)
{
    return str << "Triangle:"
               << "\n\t" << t._a << "\n\t" << t._b << "\n\t" << t._c << '\n';
}

/**
 * @brief Retrouve le troisième point dans un traingle, ayant deux autres (effectue une comparaison optimisée.)
 *
 * @param v1
 * @param v2
 * @return VertexType
 */

VertexType Triangle::third_vertex(VertexType v1, VertexType v2)
{
    bool a(false), b(false), c(false);
    if (this->get_a() == v1 || this->get_a() == v2)
    {
        a = true;
    }
    if (this->get_b() == v1 || this->get_b() == v2)
    {
        b = true;
    }
    if (this->get_c() == v1 || this->get_c() == v2)
    {
        c = true;
    }

    if (a && b)
    {
        return this->get_c();
    }
    else if (b && c)
    {
        return this->get_a();
    }
    else
    {
        return this->get_b();
    }
}

/**
 * @brief Calcule la surface d'un traingle.
 *
 * @return double
 */
double Triangle::compute_surface()
{
    double surface(0.);
    surface = 0.5 * ((this->get_b()._x - this->get_a()._x) *
                         (this->get_c()._y - this->get_a()._y) -
                     (this->get_c()._x - this->get_a()._x) *
                         (this->get_b()._y - this->get_a()._y));
    // return abs(0.5 * (((x2-x1)*(y3-y1))-((x3-x1)*(y2-y1))))
    return abs(surface);
}

/**
 * @brief S'assure que le triangle n'est pas dégénéré
 *
 */
void Triangle::check_validity()
{
    double e1 = this->get_e1().norm();
    double e2 = this->get_e2().norm();
    double e3 = this->get_e3().norm();

    std::vector<float> order;

    order.push_back(e1);
    order.push_back(e2);
    order.push_back(e3);

    std::sort(order.begin(), order.end());

    e1 = order[0];
    e2 = order[1];
    e3 = order[2];
    this->_isBad = (e3 >= 0.9 * (e1 + e2));
}

/**
 * @brief Vérifie si un point est contenu dans un triangle.
 *
 * @param pt
 * @return true
 * @return false
 */

bool Triangle::contains_point(Vec2 pt)
{
    float d1, d2, d3;
    bool has_neg, has_pos;

    d1 = sign(pt, this->get_a(), this->get_b());
    d2 = sign(pt, this->get_b(), this->get_c());
    d3 = sign(pt, this->get_c(), this->get_a());

    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}

// Compute barycentric coordinates (u, v, w) for
// point p with respect to triangle (a, b, c)
void Triangle::Barycentric(Vec2 p, float &u, float &v, float &w)
{
    Vec2 v0 = this->get_b() - this->get_a(),
         v1 = this->get_c() - this->get_a(),
         v2 = p - this->get_a();
    float d00 = v0 * v0;
    float d01 = v0 * v1;
    float d11 = v1 * v1;
    float d20 = v2 * v0;
    float d21 = v2 * v1;
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}
