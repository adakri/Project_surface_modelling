#ifndef POLYGONE_H


#include "Vec2.hpp"
#include <fstream>

#define MACRO_VARIABLE_TO_STRING(Variable) (void(Variable),#Variable)
#define debug(a) std::cout<<"DEBUG: "<<MACRO_VARIABLE_TO_STRING(a)<<" "<<a<<std::endl;

// settings
const unsigned int SCR_WIDTH = 1080;
const unsigned int SCR_HEIGHT = 720;





//put this in astruct and detailed or use eigen (not very difficult, juste qu'il faut le coder ps: j'ai commencé en anglais mais j'avais la flemme de corriger)
// Je crois que c'est mieux de mettre une classe


class Polygon
{
    private:
        std::string _polygon_fileName;
        int _N;
        bool _is_ready;
        std::vector<sf::Vertex> _edges;
        Vec2 _centroid;
    public:
        std::vector<Vec2> _points;
        //Rule of three? no pointers!
        //No default condtructor
        //constructor from points
        Polygon(std::vector<Vec2>);
        //constructor from name
        Polygon(std::string);
        //copy constructor
        Polygon(const Polygon &obj);
        //destructor
        Polygon() = default;
        ~Polygon(){};
        
        
        //implemented functions---------------------------------------------------------------
        //construct the polygon in the structure
        void construct_polygon();
        void compute_centroid();
        
        //geometry ---------------------------------------------------------------------------
        //trabslate to a point
        void translate(Vec2);
        //translate by x and y values
        void translate(spec_t, spec_t);
        void center() { Vec2 center = {0.,0.}; this -> translate(center);}
        void fit_to_window();
        void scale(float, float);
        void save_polygon();


        //Using GL/sfml --------------------------------------------------------------------------
        void draw_polygon(sf::RenderWindow&);
        //static method to manage input output ? or maybe not for encapsulation
        static void print_polygon(std::vector<Vec2>);


        //getters and setters ---------------------------------------------------------------
        std::vector<Vec2> get_points() const {return _points;};
        int get_polygon_size() const {return _N;};
        bool get_readiness() const {return _is_ready;};
        Vec2 get_barycenter() const {return _centroid;};

        void set_polygon(spec_t i, Vec2 vector ) { _points[i] = vector; };
        const void toString();
};





#define PLOYGONE_H
#endif
