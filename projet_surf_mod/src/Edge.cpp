/**
 * @file Edge.cpp
 * @author A.Vivière & A.Dakri
 * @brief Classe qui implémente une arete.
 * @version 0.1
 * @date 2022-01-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Edge.hpp"

/**
 * @brief Construct a new Edge:: Edge object
 * Constructeur
 * @param v1 
 * @param v2 
 */
Edge::Edge(const VertexType &v1, const VertexType &v2) : _v(v1), _w(v2)
{
    VertexType vi = this->get_v();
    VertexType vj = this->get_w();
    // VertexType ek = vj - vi;
    _reduced_vector = vi - vj;
}

/**
 * @brief Egalité dans le sens de la limite numérique.
 * 
 * @param e1 
 * @param e2 
 * @return true 
 * @return false 
 */
bool Edge::almost_equal(const Edge &e1, const Edge &e2)
{
    return (Vec2::almost_equal(e1.get_v(), e2.get_v()) && Vec2::almost_equal(e1.get_w(), e2.get_w())) ||
           (Vec2::almost_equal(e1.get_v(), e2.get_w()) && Vec2::almost_equal(e1.get_w(), e2.get_v()));
}

/**
 * @brief Surcharge de l'opérateur égalité.
 * 
 * @param e 
 * @return Edge& 
 */
Edge &Edge::operator=(const Edge &e)
{
    _v = e._v;
    _w = e._w;
    _reduced_vector = e._reduced_vector; // reduced vector
    _isBad = e._isBad;
    _index = e._index;
    return *this;
}

/**
 * @brief Retrouve les coefficients directeurs de la ligne passant par l'arete.
 * 
 * @param a coefficient relatif à x.
 * @param b coefficient relatif à y.
 * @param c coefficient affine.
 */
void Edge::lineFromVexrtex(spec_t &a, spec_t &b, spec_t &c)
{
    a = _v._y - _w._y;
    b = _v._x - _w._x;
    c = a * (_w._x) + b * (_v._y);
}

/**
 * @brief Retrouve la ligne bisectrice à partir d'une arete.
 * 
 * @param a 
 * @param b 
 * @param c 
 */
void Edge::perpendicularBisectorFromLine(spec_t &a, spec_t &b, spec_t &c)
{
    Vec2 mid_point = Vec2((_v._x + _w._x) / 2, (_v._y + _w._y) / 2);
    // c = -bx + ay
    c = -b * (mid_point._x) + a * (mid_point._y);
    spec_t temp = a;
    a = -b;
    b = temp;
}

/**
 * @brief retrouve le point d'intersection de deux lignes (soit la limite numérique supérieure si elles sont parallèles).
 * 
 * @param a1 |
 * @param b1 | Les coeffcients de la première droite.
 * @param c1 |
 * @param a2 |
 * @param b2 | Les coefficients de la deuxième.
 * @param c2 |
 * @return Vec2 
 */
Vec2 Edge::lineLineIntersection(spec_t a1, spec_t b1, spec_t c1,
                                spec_t a2, spec_t b2, spec_t c2)
{
    spec_t determinant = a1 * b2 - a2 * b1;
    if (determinant == 0)
    {
        // The lines are parallel. This is simplified
        // by returning a pair of FLT_MAX
        return Vec2(std::numeric_limits<spec_t>::max(), std::numeric_limits<spec_t>::max());
    }
    else
    {
        spec_t x = (b2 * c1 - b1 * c2) / determinant;
        spec_t y = (a1 * c2 - a2 * c1) / determinant;
        return Vec2(x, y);
    }
}

/**
 * @brief Surcharge de l'opérateur égalité.
 * 
 * @param e 
 * @return true 
 * @return false 
 */
bool Edge::operator==(const Edge &e) const
{
    return ((this->_v) == e._v && (this->_w) == e._w) ||
           ((this->_v) == e._w && (this->_w) == e._v);
}

/**
 * @brief Surcharge de l'opérateur de sortie standard 
 * 
 * @param str 
 * @param e 
 * @return std::ostream& 
 */
std::ostream &operator<<(std::ostream &str, const Edge &e)
{
    return str << "Edge " << e._v << ", " << e._w;
}

/**
 * @brief Calcul la norme d'une arete.
 * 
 * @return double 
 */
double Edge::norm()
{
    VertexType v = this->get_reduced_vector();
    
    return v.norm2();
}
