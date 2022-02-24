#include "Viewer.hpp"


unsigned int RESOLUTION = 0;
tgui::ListBox::Ptr popupMenu; 


/**
 * @brief Fonction qui affiche une popup lorsqu'on clique sur "Save Polygon"
 * 
 * @param gui 
 * @param position : position de la popup
 */
void rightClickCallback(tgui::GuiBase& gui, tgui::Vector2f position){
    popupMenu = tgui::ListBox::create();
    std::cout <<"entre ici"<<std::endl;
    popupMenu->addItem("Le polygone a été sauvegardé");
    popupMenu->setItemHeight(30);
    popupMenu->setPosition(position);
    popupMenu->setSize(220, popupMenu->getItemHeight() * popupMenu->getItemCount());
    gui.add(popupMenu);

}

/**
 * @brief Construct a new Viewer:: Viewer object
 * 
 * @param screen_width : largeur du viewer
 * @param screen_height : hauteur du viewer
 */
Viewer::Viewer(int screen_width, int screen_height) : _screen_width(screen_width), _screen_height(screen_height)
{
    _window = new sf::RenderWindow(sf::VideoMode(SCR_WIDTH, SCR_HEIGHT), "SFML works!");
    _window->setPosition(sf::Vector2i(500, 200));
    _window->clear(sf::Color::White);
    _gui = new tgui::Gui(*_window);
    //this->decorate();
    this->click_draw();
}

/**
 * @brief fonction qui affiche le viewer, et permet de dessiner les points
 *  et d'interagir avec la figure ou l'interface utilisateur.
 * 
 */
void Viewer::click_draw()
{   
    bool click_draw_mesh = true; 
    
    std::vector<sf::RectangleShape> rects;
    _window->clear(sf::Color::White);

    //adding buttons

    //_gui->loadWidgetsFromFile("../external_libs/form.txt");

    auto button_polygon_draw = tgui::Button::create("Draw Polygon");
    button_polygon_draw->setSize({"10%", "10%"});
    button_polygon_draw->setPosition({"5%", "5%"});
    _gui->add(button_polygon_draw);

    auto button_draw_mesh = tgui::Button::create("Draw Mesh");
    button_draw_mesh->setSize({"10%", "10%"});
    button_draw_mesh->setPosition({"5%", "20%"});
    _gui->add(button_draw_mesh);

    auto button_compute_deformation = tgui::Button::create("Apply deformation");
    button_compute_deformation->setSize({"15%", "10%"});
    button_compute_deformation->setPosition({"2%", "40%"});
    _gui->add(button_compute_deformation);

    auto button_save_polygon = tgui::Button::create("Save Polygon");
    button_save_polygon->setSize({"15%", "10%"});
    button_save_polygon->setPosition({"2%", "60%"});
    _gui->add(button_save_polygon);

    auto button_refine = tgui::Button::create("Refine Mesh");
    button_refine->setSize({"15%", "10%"});
    button_refine->setPosition({"2%", "80%"});
    _gui->add(button_refine);

    // Menu en haut du viewer
    auto menu = tgui::MenuBar::create();
    menu->setHeight(22.f);
    menu->addMenu("Mesh");
    menu->addMenu("Help");
    menu->addMenu("Exit");
    _gui->add(menu);

    // callbacks for menu items
    menu->connectMenuItem({"Exit"}, [&](){ _window->close(); });
    
    _window->clear(sf::Color::White);
    _gui->draw();

    while (_window->isOpen())
    {
        sf::Event event;

        bool file_save_open = false; // file save
        
        _window->clear(sf::Color::White);

        //Fenêtre de dessin pour délimiter l'espace de travail et les boutons.
        sf::RectangleShape rectangle;
        rectangle.setSize(sf::Vector2f(_screen_width / 1.5, _screen_height / 1.65));
        rectangle.setOutlineColor(sf::Color::Red);
        rectangle.setOutlineThickness(1);
        rectangle.setPosition(_screen_width / 6, _screen_height / 25);
        _window->draw(rectangle);

        

        while (_window->pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                _window->close();
            }
            // Actions lorsqu'on fait un clic gauche.
            if (event.type == sf::Event::MouseButtonPressed &&
                     event.mouseButton.button == sf::Mouse::Left)
            {   
                if(popupMenu)
                {
                    _gui->remove(popupMenu);
                    popupMenu = nullptr; 
                }

                // Bouton de dessin du polygon
                if(button_polygon_draw->isMouseOnWidget(tgui::Vector2f(event.mouseButton.x, event.mouseButton.y)))
                {
                    std::cout<<"Draw polygon"<<std::endl;
                    _polygon = Polygon(_points);
                    _polygon.draw_polygon(*_window);
                    std::cout<<"==========================================="<<std::endl;
                    std::cout<<"The polygon is drawn! \n";
                }

                //Bouton de sauvegarde du polygone
                if(button_save_polygon->isMouseOnWidget(tgui::Vector2f(event.mouseButton.x, event.mouseButton.y)))
                {   
                    std::cout<<"===============Saving polygon=============="<<std::endl;
                    rightClickCallback(*_gui, tgui::Vector2f(event.mouseButton.x, event.mouseButton.y));
                    _gui->handleEvent(event);
                    _gui->draw();
                    _window->display();
                    _polygon.save_polygon();
                }

                // Bouton de dessin du maillage
                if(button_draw_mesh->isMouseOnWidget(tgui::Vector2f(event.mouseButton.x, event.mouseButton.y)))
                {
                    click_draw_mesh = false; 

                    std::cout<<"Draw Mesh"<<std::endl;
                    _polygon = Polygon(_points);
                    _window->clear(sf::Color::White);
                    Polygon* temp = new Polygon(_points);
                    _mesh = Delaunay(temp,RESOLUTION);
                    draw_mesh();
                    
                    std::cout<<"==========================================="<<std::endl;
                    std::cout<<"The Mesh drawing is done! \n";
                }

                // Bouton de calcul de la déformation
                if(button_compute_deformation->isMouseOnWidget(tgui::Vector2f(event.mouseButton.x, event.mouseButton.y)))
                {
                    std::cout<<"Deform"<<std::endl;
                    _window->clear(sf::Color::White);
                    _window->draw(rectangle);
                    draw_mesh();
                }

                // Bouton d'augmentation de résolution
                if (button_refine->isMouseOnWidget(tgui::Vector2f(event.mouseButton.x, event.mouseButton.y)))
                {
                    _polygon = Polygon(_points);
                    //std::string fileName = "../data/shapes/Test.txt";
                    //_polygon = Polygon(fileName);
                    _window->clear(sf::Color::White);
                    Polygon* temp;
                    temp = &_polygon;
                    RESOLUTION ++;
                    _mesh = Delaunay(temp,RESOLUTION);
                    draw_mesh();
                
                    std::cout<<"==========================================="<<std::endl;
                    std::cout<<"The Mesh drawing is done! \n";  
                }

                if(file_save_open == true)
                {
                    debug("inside the file save")
                    file_save_open = false;
                }

                // Délimitation de la fenêtre de dessin.
                float x = sf::Vector2f(sf::Mouse::getPosition(*_window)).x;
                bool in_drawing_space =  (x <= _screen_width) && (x>= 200.);
                
                std::cout<<sf::Vector2f(sf::Mouse::getPosition(*_window)).x<<" "<<sf::Vector2f(sf::Mouse::getPosition(*_window)).y<<std::endl;

                if(in_drawing_space)
                {
                    // Dessin des points de la figure
                    if(click_draw_mesh)
                    {
                        sf::RectangleShape rect { { 3,3 } };
                        rect.setPosition(sf::Vector2f(sf::Mouse::getPosition(*_window)));
                        rect.setFillColor(sf::Color::Green);
                        rects.push_back(rect);

                        _points.push_back(VertexType(
                            sf::Vector2f(sf::Mouse::getPosition(*_window)).x,
                            sf::Vector2f(sf::Mouse::getPosition(*_window)).y
                            ));
                    }
    
                }

                for (const auto& r : rects)
                    _window->draw(r);
            }

            if (event.type == sf::Event::KeyPressed)
            {
                // Fermeture de la fenêtre
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
                {
                    _window->close();
                }

                // Fermeture de la fenêtre
                if (event.type == sf::Event::Closed)
                {
                    _window->close();
                }

                // Dessin du polygone
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::P))
                {
                    
                    _polygon = Polygon(_points);
                    //std::string fileName = "../data/shapes/Test.txt";
                    //_polygon = Polygon(fileName);
                    _polygon.draw_polygon(*_window);
                    std::cout<<"==========================================="<<std::endl;
                    std::cout<<"The polygon is drawn! \n";
                    
                }

                // Reset des points
                if(sf::Keyboard::isKeyPressed(sf::Keyboard::R)){
                    rects.clear();
                    _points.clear();
                    RESOLUTION = 0;

                    //cleaning user input
                    _handles.clear();
                    _mesh._fixedVertices.clear();
                    
                    _window->clear(sf::Color::White);
                    click_draw_mesh = true;
                }
                
                // Dessin du maillage 
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::M))
                {
                    _polygon = Polygon(_points);
                    //std::string fileName = "../data/shapes/Test.txt";
                    //_polygon = Polygon(fileName);
                    _window->clear(sf::Color::White);
                    Polygon* temp;
                    temp = &_polygon;
                    //Polygon* temp = new Polygon(_points);
                    _mesh = Delaunay(temp,RESOLUTION);
                    draw_mesh();
                
                    std::cout<<"==========================================="<<std::endl;
                    std::cout<<"The Mesh drawing is done! \n";  

                }

                // Dessin du maillage lorsque le calcul de la déformation est effectué. 
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::L)){
                    _window->clear(sf::Color::White);
                    draw_mesh();
                }

            }
            _gui->handleEvent(event);
        }
        
        // dessin des points
        for (const auto& r : rects)
            _window->draw(r); 

        _gui->draw();
        _window->display();
    }


}

/**
 * @brief Fonction qui dessine le maillage définie avec earcup
 * 
 */
void Viewer::draw_mesh(){
    int N = _mesh.getTriangles().size();
    bool whileopen = true; 
    std::vector<std::array<sf::Vertex, 2>> line(N*3);
    int i(0), j(0);
    bool def2 = true;

    // Définition de toutes les arêtes à dessiner.
    while(i<N)
    {
		line[j] = {
			sf::Vector2f(_mesh.getTriangles()[i].get_a().getX(),_mesh.getTriangles()[i].get_a().getY()),
			sf::Vector2f(_mesh.getTriangles()[i].get_b().getX(),_mesh.getTriangles()[i].get_b().getY())
					};
		line[j][0].color  = sf::Color::Black;
        line[j][1].color  = sf::Color::Black;
		j++;

		line[j] = {
			sf::Vector2f(_mesh.getTriangles()[i].get_b().getX(),_mesh.getTriangles()[i].get_b().getY()),
			sf::Vector2f(_mesh.getTriangles()[i].get_c().getX(),_mesh.getTriangles()[i].get_c().getY())
					};
		line[j][0].color  = sf::Color::Black;
        line[j][1].color  = sf::Color::Black;
		j++;

		line[j] = {
			sf::Vector2f(_mesh.getTriangles()[i].get_c().getX(),_mesh.getTriangles()[i].get_c().getY()),
			sf::Vector2f(_mesh.getTriangles()[i].get_a().getX(),_mesh.getTriangles()[i].get_a().getY())
					};
        line[j][0].color  = sf::Color::Black;
        line[j][1].color  = sf::Color::Black;
		j++;
		i++;
    }

    // Définition de cercles pour les points afin qu'ils soient cliquables par la suite. 
    std::vector<sf::CircleShape> circs;
    for(int i=0; i<_mesh.getVertices().size(); i++){
        sf::CircleShape circ(8);
        circ.setFillColor(sf::Color::Green);
        circ.setPosition(sf::Vector2f(_mesh.getVertices()[i].getX()-8, _mesh.getVertices()[i].getY()-8));
        circs.push_back(circ);
    }

    // boucle de rendu 
    while (whileopen)
    {
        sf::Clock clock;
        _window->clear(sf::Color::White);
        sf::Event event;

        //Espace de dessin
        sf::RectangleShape rectangle;
        rectangle.setSize(sf::Vector2f(_screen_width / 1.5, _screen_height / 1.65));
        rectangle.setOutlineColor(sf::Color::Red);
        rectangle.setOutlineThickness(1);
        rectangle.setPosition(_screen_width / 6, _screen_height / 25);
        _window->draw(rectangle);

        // Dessin des arêtes
        for(int i=0; i<3*N; i++)
        {
            sf::Vertex line_temp[] = {line[i][0], line[i][1]};
            _window->draw(line_temp, 2, sf::Lines);
        }

        // Dessin des poins
		for(int i=0; i<circs.size();i++)
        {
			_window->draw(circs[i]);
		}

        _gui->draw();
        _window->display();
		
        // Fermeture de la fenêtre.
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
        {
            _window->close();
        }

        // Bouton qui permet d'effacer le dessin du maillage.
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::S)){
            _handles.clear();
            _mesh._fixedVertices.clear();
			whileopen = false;
		}

        // Bouton qui permet de calculer la déformation sans mise à l'échelle.
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::C)){
			
			Eigen::VectorXd fixed_vertices(1);
   			fixed_vertices << 0;
			Deformation* deform = new Deformation(_polygon, _handles, _mesh._fixedVertices, RESOLUTION);
			deform->construct_matrix();
			Delaunay newMesh = deform->get_mesh();
            whileopen=false;
            _mesh = newMesh;

		}

        //Bouton qui permet de calculer la déformation avec mise à l'échelle.
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::V)){
			
			Eigen::VectorXd fixed_vertices(1);
   			fixed_vertices << 0;
			Deformation* deform = new Deformation(_polygon, _handles, _mesh._fixedVertices, RESOLUTION);
			deform->construct_matrix_scale();
			Delaunay newMesh = deform->get_mesh();
            whileopen=false;
            _mesh = newMesh;		
		}

        // Cache les cercles verts afin d'afficher uniquement le maillage
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::H)){
            _window->clear(sf::Color::White);
            circs.clear();
        }
        
        
        while (_window->pollEvent(event))
        {   

            //Clic qui permet de définir la déformation correspondant à un handle selectionné. 
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left && !def2)
            {
                
                float dxd = sf::Mouse::getPosition(*_window).x;
                float dyd = sf::Mouse::getPosition(*_window).y;
                _handles[_handles.size()-1].deformation_en_x = dxd;
                _handles[_handles.size()-1].deformation_en_y = dyd;
                def2 = true;

                // Dessin en bleu clair du point orientant la déformation. 
                sf::CircleShape circ(8);
                circ.setFillColor(sf::Color::Cyan);
                circ.setPosition(dxd-8, dyd-8);
                circs.push_back(circ);

            }

            //Clic qui permet de définir un handle et l'ajouter dans la liste associée.
			else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left && def2)
            {
                //On calcule la position du clic et on regarde si il se trouve dans la zone du cercle d'un des points du maillage.
				int x = sf::Mouse::getPosition(*_window).x;
				int y = sf::Mouse::getPosition(*_window).y;
				for(int i=0; i<_mesh.getVertices().size(); i++)
                {

					int dx = x - _mesh.getVertices()[i].getX();
					int dy = y - _mesh.getVertices()[i].getY();
					float d = dx*dx + dy*dy;
					if (d<=64)
                    {
                        _handles.push_back(HANDLE());
						circs[i].setFillColor(sf::Color::Red);
						_window->draw(circs[i]);
						if(std::find(_mesh._handlesIndex.begin(),_mesh._handlesIndex.end(),i)!=_mesh._handlesIndex.end())
                        {
							std::cout <<"Handle déjà ajouté"<< std::endl;
						} else
                        {
                            def2 = false;
							_mesh.setHandlesIndex(i);
                            _handles[_handles.size()-1].hand_index = i;
						}
					}
           		}	
                def = false;
			}

            // Clic qui permet de définir un fixed vertice et l'ajouter dans la liste associée. 
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Right){

                //On calcule la position du clic et on regarde si il se trouve dans la zone du cercle d'un des points du maillage.
				int x = sf::Mouse::getPosition(*_window).x;
				int y = sf::Mouse::getPosition(*_window).y;
				for(int i=0; i<_mesh.getVertices().size(); i++){
					int dx = x - _mesh.getVertices()[i].getX();
					int dy = y - _mesh.getVertices()[i].getY();
					float d = dx*dx + dy*dy;
					if (d<=64){
						circs[i].setFillColor(sf::Color::Blue);
						_window->draw(circs[i]);
						if(std::find(_mesh._fixedVertices.begin(),_mesh._fixedVertices.end(),i)!=_mesh._fixedVertices.end())
                        {
							std::cout <<"Fixed vertice déjà ajouté"<< std::endl;
						} else
                        {

                            // Ajout du fixed vertice dans la liste associée.
							_mesh.setFixedVertices(i);
						}
					}
           		}		
			}

            if (event.type == sf::Event::Closed)
                _window->close();
        }
    }


}