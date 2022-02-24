#include "Mesh.hpp"
#include "Deformation.hpp"

#include <TGUI/TGUI.hpp>




class Viewer
{
    using VertexType = Vec2;
    using EdgeType = Edge;
    using TriangleType = Triangle;

    private:
        int _screen_width;
        int _screen_height;
        sf::RenderWindow* _window;
        //drawable
        std::vector<VertexType> _points;
        Polygon _polygon;
        Delaunay _mesh;
        tgui::Gui* _gui;
        bool def;



    public:
        Viewer() = default;
        Viewer(const Viewer&) = default;
        Viewer(Viewer&&) = default;
        Viewer(int , int);
        ~Viewer(){};

        std::vector<HANDLE> _handles;

        //implemented methods
        void click_draw();
        void draw_mesh();

        //getters and setters
        int get_screen_width() const { return _screen_width;};
        int get_screen_height() const {return _screen_height;};
        Delaunay getMesh() {return _mesh;};
        Polygon getPolygon() {return _polygon;};

};


