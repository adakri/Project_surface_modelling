/**
 * @file Vec2.cpp
 * @author A.Vivière & A.Dakri
 * @brief Classe qui implémente un point 2D.
 * @version 0.1
 * @date 2022-01-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "Vec2.hpp"

/**
 * @brief Construct a new Vec 2:: Vec 2 object
 * Constructeur de la classe.
 * @param _x coordonée x
 * @param _y coordonée y
 */
Vec2::Vec2(spec_t _x, spec_t _y) : _x(_x), _y(_y)
{
}

/**
 * @brief Calcule la norme d'un point 2D.
 *
 * @return spec_t
 */
spec_t Vec2::norm2() const
{
    return sqrt(_x * _x + _y * _y);
}

/**
 * @brief Egalité au sens de la limite numérique, utile pour les approximations.
 *
 * @param x float
 * @param y float
 * @return true
 * @return false
 */
bool almost_equal(spec_t x, spec_t y)
{
    return fabs(x - y) <= std::numeric_limits<spec_t>::epsilon() * fabs(x + y) || fabs(x - y) < std::numeric_limits<spec_t>::min();
};

/**
 * @brief Egalité au sens de la limite numérique de deux points.
 *
 * @param v1
 * @param v2
 * @return true
 * @return false
 */
bool Vec2::almost_equal(const Vec2 &v1, const Vec2 &v2)
{
    return (fabs(v1.getX() - v2.getX()) <= std::numeric_limits<spec_t>::epsilon() * fabs(v1.getX() + v2.getX()) || fabs(v1.getX() - v2.getX()) < std::numeric_limits<spec_t>::min()) &&
           (abs(v1.getY() - v2.getY()) <= std::numeric_limits<spec_t>::epsilon() * fabs(v1.getY() + v2.getY()) || fabs(v1.getY() - v2.getY()) < std::numeric_limits<spec_t>::min());
};

/**
 * @brief Calcule la distance entre deux vecteurs.
 *
 * @param v Deuxième vecteur.
 * @return spec_t
 */
spec_t Vec2::dist(const Vec2 &v) const
{
    return hypotf(_x - v._x, _y - v._y);
}

/**
 * @brief Surcharge d'opérateur +/-/* pour Vec2.
 *
 * @param v Opérande.
 * @return Vec2
 */

Vec2 Vec2::operator+(const Vec2 &v)
{
    return {v._x + this->_x, v._y + this->_y};
}

Vec2 Vec2::operator-(const Vec2 &v)
{
    return {v._x - this->_x, v._y - this->_y};
}

float Vec2::operator*(const Vec2 &v)
{
    return v._x * this->_x + v._y * this->_y;
}

/**
 * @brief Surcharge de l'opératuer affectation.
 * 
 * @param v Opérande.
 * @return Vec2& 
 */

// maybe copy swap ?
Vec2 &Vec2::operator=(const Vec2 &v)
{
    _x = v._x;
    _y = v._y;
    _index = v._index;
    return *this;
}

/**
 * @brief Surcharge de l'opérateur égalité.
 * 
 * @param v 
 * @return true 
 * @return false 
 */
bool Vec2::operator==(const Vec2 &v) const
{
    return (this->_x == v._x) && (this->_y == v._y);
}

/**
 * @brief Surcharge de l'operateur de sortie standard.
 * 
 * @param str 
 * @param v 
 * @return std::ostream& 
 */
std::ostream &operator<<(std::ostream &str, const Vec2 &v)
{
    return str << "Vec2D x: " << v._x << " y: " << v._y;
}

/**
 * @brief Test si 3 points sont colinéaires.
 * 
 * @param p1 
 * @param p2 
 * @param p3 
 * @return float 
 */
// helper function
float sign(Vec2 p1, Vec2 p2, Vec2 p3)
{
    return (p1._x - p3._x) * (p2._y - p3._y) - (p2._x - p3._x) * (p1._y - p3._y);
}
