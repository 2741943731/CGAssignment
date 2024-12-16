#ifndef OBJ_INPUT_H
#define OBJ_INPUT_H

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <map>
#include "vec3.h" 

class OBJINPUT {
public:
    static bool loadOBJ(
        const std::string& filename, 
        std::vector<point3>& vertices, 
        std::vector<vec3>& normals, 
        std::vector<vec3>& texCoords, 
        std::vector<std::vector<int>>& faces,
        std::map<std::vector<int>, vec3>& materialFaces) {

        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return false;
        }

        std::string line;
        vec3 currentColor;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string type;
            iss >> type;

            if (type == "v") {
                point3 vertex;
                iss >> vertex[0] >> vertex[1] >> vertex[2];
                vertices.push_back(vertex);
            }
            else if (type == "vn") {
                vec3 normal;
                iss >> normal[0] >> normal[1] >> normal[2];
                normals.push_back(normal);
            }
            else if (type == "vt") {
                vec3 texCoord;
                iss >> texCoord[0] >> texCoord[1];
                texCoords.push_back(texCoord);
            }
            else if (type == "f") {
                std::vector<int> face;
                std::string vertexStr;
                while (iss >> vertexStr) {
                    std::istringstream vss(vertexStr);
                    std::string vertexIndexStr;
                    std::getline(vss, vertexIndexStr, '/');
                    int vertexIndex = std::stoi(vertexIndexStr) - 1; // Convert 1-based indices to 0-based
                    face.push_back(vertexIndex);
                }
                faces.push_back(face);
                //auto p = std::make_pair(face, currentColor);
                //materialFaces.insert(std::make_pair(face, currentColor));
                materialFaces[face] = currentColor;
            }
            else if (type == "usemtl") {
                std::string colorName;
                iss >> colorName;
                if (colorName == "Material__29")currentColor = vec3(42.0f / 255.0f, 199.0f / 255.0f, 172.0f / 255.0f);
                else if (colorName == "Material__27")currentColor = vec3(21.0f / 255.0f, 99.0f / 255.0f, 85.0f / 255.0f);
                else if (colorName == "Material__28")currentColor = vec3(8.0f / 255.0f, 37.0f / 255.0f, 32.0f / 255.0f);
                else if (colorName == "Material__33")currentColor = vec3(104.0f / 255.0f, 77.0f / 255.0f, 30.0f / 255.0f);
                else if (colorName == "Material__32")currentColor = vec3(73.0f / 255.0f, 54.0f / 255.0f, 21.0f / 255.0f);
                else if (colorName == "Material__35")currentColor = vec3(39.0f / 255.0f, 31.0f / 255.0f, 10.0f / 255.0f);
                else if (colorName == "Material__27")currentColor = vec3(137.0f / 255.0f, 103.0f / 255.0f, 39.0f / 255.0f);
                else if (colorName == "Material__27")currentColor = vec3(31.0f / 255.0f, 138.0f / 255.0f, 120.0f / 255.0f);
                else if (colorName == "Material__27")currentColor = vec3(51.0f / 255.0f, 234.0f / 255.0f, 203.0f / 255.0f);
            }
        }
        file.close();

        return true;
    }
};

#endif
