#pragma once
class Display
{
public:
	Display();
	virtual ~Display();
	void Create(
		int width,
		int height,
		char *windowName,
		double red,
		double green,
		double blue);
	void Run();
	vtkSmartPointer<vtkRenderer> GetRenderer();

public:
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style;
};

