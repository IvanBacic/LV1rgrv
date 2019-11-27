#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include "Display.h"


Display::Display()
{
}


Display::~Display()
{
}

void Display::Create(
	int width,
	int height,
	char *windowName,
	double red,
	double green,
	double blue)
{
	renderer = vtkSmartPointer<vtkRenderer>::New();
	renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(width, height);
	renderWindow->SetWindowName(windowName);
	renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	renderWindowInteractor->SetInteractorStyle(style);
	renderer->SetBackground(red, green, blue);
}

void Display::Run()
{
	renderWindow->Render();
	renderWindowInteractor->Start();
}

vtkSmartPointer<vtkRenderer> Display::GetRenderer()
{
	return renderer;
}

