#include "Polygon.hpp"
//https://stackoverflow.com/questions/3589422/using-opengl-glutdisplayfunc-within-class


/**
 * @brief Construct a new Polygon:: Polygon object
 * 
 * @param points : points du polygone.
 */
Polygon::Polygon(std::vector<Vec2> points): _points(points), _is_ready(true), _polygon_fileName("Nofile")
{
    _N = _points.size();
}

/**
 * @brief Construct a new Polygon:: Polygon object
 * 
 * @param obj: copie d'un polygone existant.
 */
Polygon::Polygon(const Polygon &obj) : _polygon_fileName(obj._polygon_fileName), _N(obj._N), _points(obj._points), _centroid(obj._centroid), _is_ready(true)
{

}

/**
 * @brief Construct a new Polygon:: Polygon object
 * 
 * @param polygon_fileName : création d'un polygone à partir d'un fichier. 
 */
Polygon::Polygon(std::string polygon_fileName): _polygon_fileName(polygon_fileName), _is_ready(false), _N(0)
{
    this -> construct_polygon();
    _is_ready = true;
    _N = _points.size();
    _edges.resize(_N);
    this->compute_centroid();
    this->fit_to_window();
}

/**
 * @brief Calcule le centre du polygone. 
 * 
 */
void Polygon::compute_centroid()
{
    _centroid = Vec2(0.,0.);
    for(int i=0; i<_N; i++)
    {
        _centroid._x += _points[i]._x;
        _centroid._y += _points[i]._y;
    }
    _centroid._x = _centroid._x  / _N;
    _centroid._y = _centroid._y / _N;  
}

/**
 * @brief Création du polygone à partir d'un fichier
 * 
 */
void Polygon::construct_polygon()
{
    std::ifstream dataFile(_polygon_fileName.data());
    if(!dataFile.is_open())
    {
        std::cout << "Unable to open file " << _polygon_fileName << std::endl;
        std::cout<<"====================================================="<<std::endl;
        abort();
    }
    else
    {
        std::cout << "Reading data file " << _polygon_fileName << std::endl;
        std::cout<<"====================================================="<<std::endl;
    }

    std::string fileLine;
    double tmp1,tmp2;
    bool stop(false);
    int i(0);

    while (std::getline(dataFile, fileLine) && !stop)
    {   
        dataFile >> tmp1 >> tmp2;
        _points.push_back({tmp1,tmp2});
    }
    std::cout<<"====================================================="<<std::endl;
    std::cout<<"File reading complete!"<<std::endl;

    // Calcul du centre du polygone
    for (unsigned i=0; i<_points.size(); i++)
    {
        _centroid._x += _points[i]._x;
        _centroid._y += _points[i]._y;
    }

    _centroid._x /= _N;
    _centroid._y /= _N;

    std::cout<<"====================================================="<<std::endl;
    std::cout<<"Processing complete!"<<std::endl;
}

/**
 * @brief Affichage des points du polygone
 * 
 */
const void Polygon::toString()
{
    print_polygon(_points);
}

/**
 * @brief Affiche des points du polygone
 * 
 * @param polygon : points
 */
void Polygon::print_polygon(std::vector<Vec2> polygon)
{

    std::cout<<"============================================"<<std::endl;
    std::cout<<"Polygon of size "<<polygon.size()<<std::endl;
    for(unsigned i=0; i<polygon.size(); i++)
    {
        std::cout<<polygon[i]._x<<";"<<polygon[i]._x<<std::endl;
    }

}

/**
 * @brief Mise à l'échelle des points du polygone.
 * 
 * @param scale_x : par rapport à x
 * @param scale_y : par rapport à y
 */
void Polygon::scale(float scale_x, float scale_y )
{
    for(int i=0; i<_N; i++)
    {
        _points[i]._x *= scale_x;
        _points[i]._y *= scale_y;
    }
}

/**
 * @brief Translation du polygone
 * 
 * @param _x 
 * @param _y 
 */
void Polygon::translate(spec_t _x, spec_t _y)
{
    for(unsigned i=0; i<_N; i++)
    {
        Vec2 v = {_points[i]._x + _x, _points[i]._y + _y};
        _points[i] = v;        
    }
}

/**
 * @brief Translation selon un vecteur.
 * 
 * @param v : vecteur à partir duquel se fait la translation.
 */
void Polygon::translate(Vec2 v)
{
    Vec2 vector = v -_centroid;
    this -> translate(vector._x, vector._y);
}

/**
 * @brief Mise à l'échelle des points pour que le polygone rentre dans la fenêtre.
 * 
 */
void Polygon::fit_to_window()
{
    spec_t displacementx = abs(_centroid._x - (SCR_WIDTH / 2.) );
    spec_t displacementy = abs(_centroid._y - (SCR_HEIGHT / 2.) );

    for(int i=0; i<_N; i++)
    {
        float new_x,new_y;
        new_x =  _points[i].getX()  + displacementx;
        new_y =  _points[i].getY()  + displacementy; 
        _points[i] = Vec2(new_x, new_y);
    }
    
}

/**
 * @brief Fonction qui définit les éléments du polygone à dessine puis les dessine.
 * 
 * @param window : fenêtre SFML qui accueille le polygone. 
 */
void Polygon::draw_polygon(sf::RenderWindow& window)
{
    
    // Construction des arêtes.
    bool whileopen = true;
    std::vector<std::array<sf::Vertex, 2>> line(_N);
    std::cout<<"==========================================="<<std::endl;
    for(int i=0; i<_N; i++)
    {
        //std::cout<<"["<<_points[i]._x <<" "<< _points[i]._y<<"] ";
        if(i==_N-1)
        {
            line[i] = {sf::Vertex(sf::Vector2f(_points[i]._x, _points[i]._y)),
                sf::Vertex(sf::Vector2f(_points[0]._x, _points[0]._y))};
        }else{
            line[i] = {sf::Vertex(sf::Vector2f(_points[i]._x, _points[i]._y)),
                sf::Vertex(sf::Vector2f(_points[i+1]._x, _points[i+1]._y))};
        }
        
        line[i][0].color  = sf::Color::Black;
        line[i][1].color  = sf::Color::Black;
    }
    // Cercle qui représente les points
    std::vector<sf::CircleShape> circs;
    for(int i=0; i<_N; i++)
    {
        sf::CircleShape circ(8);
        circ.setFillColor(sf::Color::Green);
        circ.setPosition(sf::Vector2f(_points[i].getX()-8, _points[i].getY()-8));
        circs.push_back(circ);
    }

    std::cout<<"The polygon drawing is done! \n";
    std::cout<<"==========================================="<<std::endl;
    window.display();

    // Boucle de rendu.
    while (whileopen)
    {
        sf::Clock clock;
        sf::Event event;
        sf::Vertex line_temp[2];

        for(int i=0; i<_N+1; i++)
        {
            sf::Vertex line_temp[] = {line[i][0], line[i][1]};
            window.draw(line_temp, 2, sf::Lines);
            
        }
        for(int i=0;i<circs.size();i++)
        {
            window.draw(circs[i]);
        }

        window.display();
       
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
        {
            window.close();
        }
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))
        {
            whileopen = false;
        }
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        } 
    } 
   
}

// Sauvegarde du polygone dans un fichier .txt ce qui permet de l'ouvrir plus tard. 
void Polygon::save_polygon()
{
    if(this->_points.size() == 0)
    {
        std::cout<<"Draw the polygon first"<<std::endl;
    }else{
        std::ofstream myfile;
        myfile.open ("../data/shapes/tmp_polygon.txt");
        for(int i=0; i<this->_points.size(); i++)
        {
            myfile << _points[i]._x << " " << _points[i]._y << "\n";
        }
        myfile.close();
    }
} 
