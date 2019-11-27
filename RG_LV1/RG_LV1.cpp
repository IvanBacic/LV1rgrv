#include "stdafx.h"
#include <Windows.h>
#include <ctime>

#include <conio.h>
#include <vector>
#include <math.h>

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAxesActor.h>
#include <vtkTextActor.h>
#include <vtkTransform.h>

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPolyDataNormals.h>
#include <vtkFloatArray.h>
#include <vtkIdList.h>
#include <vtkCallbackCommand.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkLandmarkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPLYReader.h>

#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkTextProperty.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
//#include <opencv2/highgui.hpp>

#include <opencv2/objdetect.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>


#include "RVL3DTools.h"
#include "Display.h"
#include "Body.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

#define PI 3.14159265359


struct mouseCallbackDataLV2
{
	Mat image;
	Subdiv2D *pSubdiv;
	int numOfEdges;
};

void Delaunaytriangulation(Subdiv2D *subdiv,int margins, int mapDimenzion,int numOfPoints);

int GetNumOfEdges(int numOfPoints,Subdiv2D subdiv);

void DrawDelaunay(int numOfEdges,Subdiv2D subdiv,Mat image);

void LV2MouseCallback(int event, int x, int y, int flags, void* userdata);


bool insideOfTriangleForFirstTime(int secondEdgeIndex, int  firstPointIndex, int currentEdgeIndex, Subdiv2D subdiv);

int main(int argc, char* argv[])
{
	int numOfPoints = 40;
	int mapDimenzion = 600;
	int margins = 20;
	int imageDimenzion = mapDimenzion + 2 * margins;

	Mat image;
	image.create(imageDimenzion, imageDimenzion, CV_8UC3);
	image.setTo(255);

	Rect rectangle(0, 0, imageDimenzion, imageDimenzion);
	Subdiv2D subdiv(rectangle);

	Delaunaytriangulation(&subdiv, margins, mapDimenzion, numOfPoints);
	
	int numOfEdges;
	numOfEdges = GetNumOfEdges(numOfPoints, subdiv);

	DrawDelaunay(numOfEdges,subdiv,image);

	mouseCallbackDataLV2 callbackData;
	callbackData.image = image;
	callbackData.pSubdiv = &subdiv;
	callbackData.numOfEdges = numOfEdges;

	setMouseCallback("Map", LV2MouseCallback, &callbackData);

	imshow("Map", image);
	waitKey();

	setMouseCallback("Map", NULL, NULL); // if somthing is clicked in console destry window
	destroyWindow("Map");

	return 0;
}



void Delaunaytriangulation(Subdiv2D* subdiv, int margins, int mapDimenzion,int numOfPoints) {
	vector<Point2f> randPoints;

	randPoints.push_back(Point2f(margins, margins));
	randPoints.push_back(Point2f(margins, margins + mapDimenzion));
	randPoints.push_back(Point2f(margins + mapDimenzion, margins + mapDimenzion));
	randPoints.push_back(Point2f(margins + mapDimenzion, margins));


	for (int i = 4; i < numOfPoints; i++) {
		randPoints.push_back(Point2f(margins + rand() % mapDimenzion, margins + rand() % mapDimenzion));
	}

	for (vector<Point2f>::iterator iter = randPoints.begin(); iter != randPoints.end(); iter++) {
		subdiv->insert(*iter);
	}
}



int GetNumOfEdges(int numOfPoints, Subdiv2D subdiv) {

	int numOfEdges = 0, currentEdgeIndex = 0, secondEdgeIndex;

	for (int i = 4; i < numOfPoints; i++) {
		subdiv.getVertex(i, &currentEdgeIndex);
		secondEdgeIndex = currentEdgeIndex;

		do {

			if (secondEdgeIndex > numOfEdges) {  //if secondedgeIndex is bigger than the number of edges update our numOfEdges
				numOfEdges = secondEdgeIndex;
			}

			secondEdgeIndex = subdiv.getEdge(secondEdgeIndex, subdiv.NEXT_AROUND_ORG);
			// secondEdgeIndex becomes next edge around org and continou only if new secondEdgeIndex is different from currentEdgeIndex
		} while (secondEdgeIndex != currentEdgeIndex);
	}
	numOfEdges++;
	
	return numOfEdges;
}


void DrawDelaunay(int numOfEdges, Subdiv2D  subdiv, Mat image) {

	int currentEdgeIndex = 0;
	int firstPointIndex, secondPointIndex;
	Point fisrtPoint, secondPoint;

	for (currentEdgeIndex = 0; currentEdgeIndex < numOfEdges; currentEdgeIndex++)
	{
		firstPointIndex = subdiv.edgeOrg(currentEdgeIndex);
		secondPointIndex = subdiv.edgeDst(currentEdgeIndex);

		if (firstPointIndex >= 4 && secondPointIndex > firstPointIndex)
		{
			fisrtPoint = subdiv.getVertex(firstPointIndex);
			secondPoint = subdiv.getVertex(secondPointIndex);

			line(image, fisrtPoint, secondPoint, Scalar(0, 255, 0));
		}
	}

	Mat imageD = image.clone();
	imshow("Map", imageD);
	imageD.release();
}

bool insideOfTriangleForFirstTime(int secondEdgeIndex,int  firstPointIndex,int currentEdgeIndex,Subdiv2D subdiv) {
	int secondPointIndex;

	do
	{
		secondPointIndex = subdiv.edgeOrg(secondEdgeIndex);

		if (secondPointIndex < firstPointIndex || secondPointIndex < 4) // check if we alredy visited triangle
		{
			return false;
		}

		secondEdgeIndex = subdiv.getEdge(secondEdgeIndex, subdiv.NEXT_AROUND_LEFT);
	} while (secondEdgeIndex != currentEdgeIndex);

	return true;
}

void LV2MouseCallback(int event, int x, int y, int flags, void* userdata)
{
	mouseCallbackDataLV2* pUserData = (mouseCallbackDataLV2*)userdata;

	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		float mouseX = (float)x;
		float mouseY = (float)y;

		// all variable needed in function
		int currentEdgeIndex, secondEdgeIndex, thirdEdgeIndex, firstPointIndex, secondPointIndex, vertexIndex, edgeIterator;
		Point2f firstPoint, secondPoint;
		bool bValidAndVisitedForTheFirstTime, bInsideTriangle, bValidTriangle;
		float nx, ny;
		Point vertexArray[3];
		int indexEdgeArray[3];
		Mat image;
		Subdiv2D *subdiv = pUserData->pSubdiv;


		for (currentEdgeIndex = 0; currentEdgeIndex < pUserData->numOfEdges; currentEdgeIndex++)
		{
			firstPointIndex = pUserData->pSubdiv->edgeOrg(currentEdgeIndex);

			if (firstPointIndex < 4) // if our edge is one of the corner edges
				continue;


			secondEdgeIndex = currentEdgeIndex;
			bValidAndVisitedForTheFirstTime = insideOfTriangleForFirstTime(secondEdgeIndex, firstPointIndex, currentEdgeIndex, *subdiv);


			if (!bValidAndVisitedForTheFirstTime)
				continue;

			bInsideTriangle = true;

			vertexIndex = 0;

			edgeIterator = 0;

			secondEdgeIndex = currentEdgeIndex;

			do
			{
				firstPointIndex = pUserData->pSubdiv->edgeOrg(secondEdgeIndex);
				secondPointIndex = pUserData->pSubdiv->edgeDst(secondEdgeIndex);

				firstPoint = pUserData->pSubdiv->getVertex(firstPointIndex); //first point of edge (org)
				secondPoint = pUserData->pSubdiv->getVertex(secondPointIndex); // second point of edge (dest)

				vertexArray[vertexIndex++] = firstPoint;  // adding points of current triangle

				indexEdgeArray[edgeIterator++] = secondEdgeIndex;

				nx = firstPoint.y - secondPoint.y;  // calculets normal of our edge
				ny = secondPoint.x - firstPoint.x;

				if (nx * (mouseX - firstPoint.x) + ny * (mouseY - firstPoint.y) <= 0.0f) // if it isn't under of of the edges of current triangle
				{																	     // we are not in thaht triangle
					bInsideTriangle = false;
					break;
				}

				secondEdgeIndex = pUserData->pSubdiv->getEdge(secondEdgeIndex, pUserData->pSubdiv->NEXT_AROUND_LEFT); // check next left edge of current triangle

			} while (secondEdgeIndex != currentEdgeIndex); // until we get to starting triangle



			if (bInsideTriangle) // if we are in valid triangle color it
			{
				image = pUserData->image.clone(); // so we dont overwrite orriginal image

				fillConvexPoly(image, vertexArray, 3, Scalar(100, 20, 200));

				for (int i = 0; i < 3; i++)
				{
					thirdEdgeIndex = pUserData->pSubdiv->rotateEdge(indexEdgeArray[i], 2); // go to neighbour 

					vertexIndex = 0;

					bValidTriangle = true;

					secondEdgeIndex = thirdEdgeIndex;

					do
					{
						firstPointIndex = pUserData->pSubdiv->edgeOrg(secondEdgeIndex); 

						if (firstPointIndex < 4)  // chec if thaht neighbour is valid (not part of first for points)
						{
							bValidTriangle = false;
							break;
						}
						vertexArray[vertexIndex++] = pUserData->pSubdiv->getVertex(firstPointIndex); // if valid add  it to triangle to color
						secondEdgeIndex = pUserData->pSubdiv->getEdge(secondEdgeIndex, pUserData->pSubdiv->NEXT_AROUND_LEFT); // go to next
					} while (secondEdgeIndex != thirdEdgeIndex);
					if (bValidTriangle)
						fillConvexPoly(image, vertexArray, 3, Scalar(255, 100, 10)); // color that triangle
				}

				imshow("Map", image);

				image.release();
			}
		}
	}
}