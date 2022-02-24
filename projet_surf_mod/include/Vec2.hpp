#ifndef VEC2_H
#define VEC2_H

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sstream>
#include <stdint.h>
#include <chrono>
#include <ctime> 

#include "delaunator_ext.hpp"

#define spec_t double


class Vec2 {

    private:
      //Vec2 est une friend (au sens lèger du mot) vaut mieux mettre tout en public
    public:
      spec_t _x;
      spec_t _y;
      int _index;
      //Première fois que j'avais besoin d'implémenter un constructeur par défault et que je ne l'ai pas implementé
      Vec2()=default;
      Vec2(spec_t x, spec_t y);
      ~Vec2(){};

      //implemented methods
      spec_t norm2() const;
      static bool almost_equal(const Vec2 &, const Vec2 &);
      spec_t dist(const Vec2 &) const;

      Vec2 operator+(const Vec2& v);
      Vec2 operator-(const Vec2& v);
      float operator*(const Vec2& v);
      bool operator==(const Vec2& v) const;
      Vec2& operator=(const Vec2& v);
      
      friend std::ostream &operator <<(std::ostream &str, const Vec2 &v);
      void isString() const{
          std::cout<<"vector: _x:"<< _x <<" , y: "<< _y << " index: " << _index << std::endl;
      };


      // getter/setter
      spec_t getX() const { return _x;};
      spec_t getY() const { return _y;};
      int get_index() const { return _index;};
      void setX(spec_t x) {_x = x;};
      void setY(spec_t y) {_y = y;};
      
};

float sign(Vec2 , Vec2 , Vec2);


#endif
