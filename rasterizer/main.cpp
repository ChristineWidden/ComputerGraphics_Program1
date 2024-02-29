/* Release code for program 1 CPE 471 Fall 2016 */

#include <iostream>
#include <algorithm>
#include <math.h>
#include <string>
#include <vector>
#include <memory>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "Image.h"

// This allows you to skip the `std::` in front of C++ standard library
// functions. You can also say `using std::cout` to be more selective.
// You should never do this in a header file.
using namespace std;

int g_width, g_height;

/*
   Helper function you will want all quarter
   Given a vector of shapes which has already been read from an obj file
   resize all vertices to the range [-1, 1]
 */
void resize_obj(std::vector<tinyobj::shape_t> &shapes){
   float minX, minY, minZ;
   float maxX, maxY, maxZ;
   float scaleX, scaleY, scaleZ;
   float shiftX, shiftY, shiftZ;
   float epsilon = 0.001;

   minX = minY = minZ = 1.1754E+38F;
   maxX = maxY = maxZ = -1.1754E+38F;

   //Go through all vertices to determine min and max of each dimension
   for (size_t i = 0; i < shapes.size(); i++) {
      for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
         if(shapes[i].mesh.positions[3*v+0] < minX) minX = shapes[i].mesh.positions[3*v+0];
         if(shapes[i].mesh.positions[3*v+0] > maxX) maxX = shapes[i].mesh.positions[3*v+0];

         if(shapes[i].mesh.positions[3*v+1] < minY) minY = shapes[i].mesh.positions[3*v+1];
         if(shapes[i].mesh.positions[3*v+1] > maxY) maxY = shapes[i].mesh.positions[3*v+1];

         if(shapes[i].mesh.positions[3*v+2] < minZ) minZ = shapes[i].mesh.positions[3*v+2];
         if(shapes[i].mesh.positions[3*v+2] > maxZ) maxZ = shapes[i].mesh.positions[3*v+2];
      }
   }

	//From min and max compute necessary scale and shift for each dimension
   float maxExtent, xExtent, yExtent, zExtent;
   xExtent = maxX-minX;
   yExtent = maxY-minY;
   zExtent = maxZ-minZ;
   if (xExtent >= yExtent && xExtent >= zExtent) {
      maxExtent = xExtent;
   }
   if (yExtent >= xExtent && yExtent >= zExtent) {
      maxExtent = yExtent;
   }
   if (zExtent >= xExtent && zExtent >= yExtent) {
      maxExtent = zExtent;
   }
   scaleX = 2.0 /maxExtent;
   shiftX = minX + (xExtent/ 2.0);
   scaleY = 2.0 / maxExtent;
   shiftY = minY + (yExtent / 2.0);
   scaleZ = 2.0/ maxExtent;
   shiftZ = minZ + (zExtent)/2.0;

   //Go through all verticies shift and scale them
   for (size_t i = 0; i < shapes.size(); i++) {
      for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
         shapes[i].mesh.positions[3*v+0] = (shapes[i].mesh.positions[3*v+0] - shiftX) * scaleX;
         assert(shapes[i].mesh.positions[3*v+0] >= -1.0 - epsilon);
         assert(shapes[i].mesh.positions[3*v+0] <= 1.0 + epsilon);
         shapes[i].mesh.positions[3*v+1] = (shapes[i].mesh.positions[3*v+1] - shiftY) * scaleY;
         assert(shapes[i].mesh.positions[3*v+1] >= -1.0 - epsilon);
         assert(shapes[i].mesh.positions[3*v+1] <= 1.0 + epsilon);
         shapes[i].mesh.positions[3*v+2] = (shapes[i].mesh.positions[3*v+2] - shiftZ) * scaleZ;
         assert(shapes[i].mesh.positions[3*v+2] >= -1.0 - epsilon);
         assert(shapes[i].mesh.positions[3*v+2] <= 1.0 + epsilon);
      }
   }
}

class Vertex3d {
public:
	float x, y, z;
};

class BoundingBox {
public:
	int minX() {
		return minx;
	}
	int minY() {
		return miny;
	}
	int maxX() {
		return maxx;
	}
	int maxY() {
		return maxy;
	}

	BoundingBox(Vertex3d A, Vertex3d B, Vertex3d C, int maxWidth, int maxHeight) {
		int minWidth = 0;
		int minHeight = 0;

		pair<float, float> minmaxX = minmax({ A.x, B.x, C.x });
		minx = max(roundf(minmaxX.first), float(minWidth));
		maxx = min(roundf(minmaxX.second), float(maxWidth));
		//minx = minmaxX.first;
		//maxx = minmaxX.second;
		pair<float, float> minmaxY = minmax({ A.y, B.y, C.y });
		miny = max(roundf(minmaxY.first), float(minHeight));
		maxy = min(roundf(minmaxY.second), float(maxHeight));
		//miny = minmaxY.first;
		//maxy = minmaxY.second;
	}

private:
	float minx;
	float miny;
	float maxx;
	float maxy;
};

float triangleAreaVertices(Vertex3d A, Vertex3d B, Vertex3d C) {
	double x1, y1, x2, y2, x3, y3;
	x1 = A.x;
	y1 = A.y;
	x2 = B.x;
	y2 = B.y;
	x3 = C.x;
	y3 = C.y;
	return abs( ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2.0);
}

double triangleAreaPairedCoords(vector<double> triangle) {
	double x1, y1, x2, y2, x3, y3;
	x1 = triangle[0];
	y1 = triangle[1];
	x2 = triangle[2];
	y2 = triangle[3];
	x3 = triangle[4];
	y3 = triangle[5];
	return abs( ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2.0);
}

/**
* returns areas of 3 subtriangles
**/
vector<double> pointInTriangle(float triangleArea, float x, float y, Vertex3d vA, Vertex3d vB, Vertex3d vC) {

	double x1, y1, x2, y2, x3, y3;
	x1 = vA.x;
	y1 = vA.y;
	x2 = vB.x;
	y2 = vB.y;
	x3 = vC.x;
	y3 = vC.y;

	double A1 = triangleAreaPairedCoords({ x, y, x2, y2, x3, y3 });
	double A2 = triangleAreaPairedCoords({ x, y, x1, y1, x3, y3 });
	double A3 = triangleAreaPairedCoords({ x, y, x1, y1, x2, y2 });

	if (abs(A1 + A2 + A3 - triangleArea) < 0.1) {
		return { A1, A2, A3 };
	}
	else {
		return {};
	}
}

int main(int argc, char **argv)
{


	// OBJ filename
	string meshName("resources/bunny.obj");
	string imgName("target.png");

	//set g_width and g_height appropriately!
	g_width = g_height = 2083;

   //create an image
	auto image = make_shared<Image>(g_width, g_height);

	short** zBuf = new short* [g_width];
	for (size_t i = 0; i < g_width; i++)
	{
		zBuf[i] = new short[g_height];
		for (size_t j = 0; j < g_height; j++)
		{
			zBuf[i][j] = 0;
		}
	}

	// triangle buffer
	vector<unsigned int> triBuf; 
	// position buffer
	vector<float> posBuf;
	// Some obj files contain material information.
	// We'll ignore them for this assignment.
	vector<tinyobj::shape_t> shapes; // geometry
	vector<tinyobj::material_t> objMaterials; // material
	string errStr;
	
   bool rc = tinyobj::LoadObj(shapes, objMaterials, errStr, meshName.c_str());
	/* error checking on read */
	if(!rc) {
		cerr << errStr << endl;
	} else {
 		//keep this code to resize your object to be within -1 -> 1

	
   	resize_obj(shapes); 
		posBuf = shapes[0].mesh.positions;
		triBuf = shapes[0].mesh.indices;
	}


	cout << "Number of vertices: " << posBuf.size()/3 << endl;
	cout << "Number of triangles: " << triBuf.size()/3 << endl;

	//TODO add code to iterate through each triangle and rasterize it 





	for (size_t i = 0; i < posBuf.size(); i += 3)
	{
		//resize to canvas
		posBuf[i] = roundf((posBuf[i] + 1) * image->getWidth() * 0.5);
		posBuf[i+1] = roundf((posBuf[i + 1] + 1) * image->getHeight() * 0.5);

		//convert z to color
		posBuf[i + 2] = (posBuf[i + 2] + 1) * 255 * 0.5;
	}

	for (size_t i = 0; i < triBuf.size(); i += 3)
	{
		// go through every pointer to a vertex in tribuf, in groups of 3
		
		Vertex3d a, b, c;

		int vertexPointer = triBuf[i];
		a.x = posBuf[3 * vertexPointer];
		a.y = posBuf[3 * vertexPointer + 1];
		a.z = posBuf[3 * vertexPointer + 2];

		vertexPointer = triBuf[i+1];
		b.x = posBuf[3 * (vertexPointer)];
		b.y = posBuf[3 * (vertexPointer) + 1];
		b.z = posBuf[3 * (vertexPointer) + 2];

		vertexPointer = triBuf[i+2];
		c.x = posBuf[3 * (vertexPointer)];
		c.y = posBuf[3 * (vertexPointer) + 1];
		c.z = posBuf[3 * (vertexPointer) + 2];

		double triangleArea = triangleAreaVertices(a, b, c);

		BoundingBox boundingBox(a, b, c, g_width, g_height);

		for (int i = boundingBox.minX(); i < boundingBox.maxX(); i++)
		{
			for (int j = boundingBox.minY(); j < boundingBox.maxY(); j++)
			{
				vector<double> result = pointInTriangle(triangleArea, i, j, a, b, c);
				if (result.size() == 3) {

					int r = floor((a.z * result[0] / triangleArea)
								+ (b.z * result[1] / triangleArea)
								+ (c.z * result[2] / triangleArea));

					if (r > 255) r = 255;

					if (r > zBuf[i][j] ) {
						zBuf[i][j] = r;

					}
					
					r = zBuf[i][j];
					image->setPixel(i, j, r, 0, 0);
					
				}
			}
		}
	}

	
	//write out the image
   image->writeToFile(imgName);

	return 0;
}
