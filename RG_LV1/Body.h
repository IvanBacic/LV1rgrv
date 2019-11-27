#include <vtkActorCollection.h>
#include <vtkPolygon.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkSphereSource.h>
#include <vtkConeSource.h>
#include <vtkLine.h>
#include <vtkVectorText.h>
#include <vtkLabeledDataMapper.h>
#include <vtkTextMapper.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>

#pragma once
class Body
{
public:
	Body();
	virtual ~Body();
	void CreateRectangle(
		double a,
		double b,
		double red,
		double green,
		double blue);
	void CreateBox(
		double a,
		double b,
		double c,
		double red,
		double green,
		double blue);
	void CreateCylinder(
		double r,
		double h,
		int resolution,
		double red,
		double green,
		double blue);
	void CreateSphere(
		double r,
		int resolution,
		double red,
		double green,
		double blue);
	void CreateCone(
		double r,
		double h,
		int resolution,
		double red,
		double green,
		double blue);
	void CreateLine(
		double *P1,
		double *P2,
		double red,
		double green,
		double blue);
	void CreateDashedLine(
		double *P1,
		double *P2,
		double onLength,
		double offLength,
		double red,
		double green,
		double blue);
	void CreateText(
		char *text,
		double red,
		double green,
		double blue);
	void SetCaption(		
		char *text,
		double *P,
		double *offset,
		double red,
		double green,
		double blue,
		double fontSize);
	void AddPart(Body * pPart);
	void Translate(double x, double y, double z);
	void Transform(double *T);
	void RotX(double q);
	void RotY(double q);
	void RotZ(double q);
	void AddToRenderer(
		vtkSmartPointer<vtkRenderer> renderer,
		vtkSmartPointer<vtkTransform> T = NULL);
	void AddToDisplay(
		Display *pWindow,
		vtkSmartPointer<vtkTransform> T = NULL);
	void SetOpacity(double opacity);
	void SetCaptionOffset(double x, double y);
	void SetLineWidth(double width);
	void LoadFromFile(
		char *fileName,
		bool bComputeNormals = false);

public:
	std::vector<Body *> parts;
	Body *parent;

private:
	vtkSmartPointer<vtkActor> actor;
	vtkSmartPointer<vtkTransform> pose;
	vtkSmartPointer<vtkCaptionActor2D> caption;
	double captionAttachmentPoint[3];
};

