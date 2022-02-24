#include <iostream>
#include <fstream>
#include <chrono>

#include "Viewer.hpp"
#include "Deformation.hpp"

// Redecalration
static std::vector<Vec2> points;

/**
 * @brief Checks the command line if .txt file is inputed.
 * 
 * @param argc 
 * @param argv 
 * @param fileName The name of the file
 */

void check_cmd_input(int argc, char **argv, std::string &fileName)
{
    if (argc == 1)
    {
        std::cout << "Please input the file name" << std::endl;
        exit(1);
    }
    else if (argc > 2)
    {
        std::cout << "Please input less args, args count is " << argc << std::endl;
        exit(1);
    }
    else
    {
        fileName = argv[1];
    }
}

/**
 * @brief Main function 
 * 
 * @param argc argument count: 1
 * @param argv empty
 * @return int Exit status 
 */

int main(int argc, char **argv)
{

    Viewer *viewer = new Viewer(1280, 1080);

    return EXIT_SUCCESS;
}
