#include <vector>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataNormals.h>
#include "Display.h"
#include "Body.h"
#include "RVL3DTools.h"


Body::Body()
{
	pose = vtkSmartPointer<vtkTransform>::New();

	pose->PostMultiply();

	actor = NULL;
	caption = NULL;
}


Body::~Body()
{
}

void Body::CreateRectangle(
	double a,
	double b,
	double red,
	double green,
	double blue)
{
	// Setup four points
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(0.5 * a, 0.5 * b, 0.0);
	points->InsertNextPoint(-0.5 * a, 0.5 * b, 0.0);
	points->InsertNextPoint(-0.5 * a, -0.5 * b, 0.0);
	points->InsertNextPoint(0.5 * a, -0.5 * b, 0.0);

	// Create the polygon
	vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
	polygon->GetPointIds()->SetNumberOfIds(4); //make a quad
	polygon->GetPointIds()->SetId(0, 0);
	polygon->GetPointIds()->SetId(1, 1);
	polygon->GetPointIds()->SetId(2, 2);
	polygon->GetPointIds()->SetId(3, 3);

	// Add the polygon to a list of polygons
	vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
	polygons->InsertNextCell(polygon);

	// Create a PolyData
	vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
	polygonPolyData->SetPoints(points);
	polygonPolyData->SetPolys(polygons);

	// Create a mapper and actor.
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polygonPolyData);
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(red, green, blue);
}

void Body::CreateBox(
	double a,
	double b,
	double c,
	double red,
	double green,
	double blue
	)
{
	// Create a cube.
	vtkSmartPointer<vtkCubeSource> cubeSource =
		vtkSmartPointer<vtkCubeSource>::New();
	cubeSource->SetXLength(a);
	cubeSource->SetYLength(b);
	cubeSource->SetZLength(c);

	// Create a mapper and actor.
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(cubeSource->GetOutputPort());
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(red, green, blue);
}

void Body::CreateCylinder(
	double r,
	double h,
	int resolution,
	double red,
	double green,
	double blue)
{
	// Create a cylinder
	vtkSmartPointer<vtkCylinderSource> cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
	//cylinderSource->SetCenter(0.0, 0.0, 0.0);
	cylinderSource->SetRadius(r);
	cylinderSource->SetHeight(h);
	cylinderSource->SetResolution(resolution);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(cylinderSource->GetOutputPort());
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(red, green, blue);	
}

void Body::CreateSphere(
	double r,
	int resolution,
	double red,
	double green,
	double blue)
{
	// Create a cone
	vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	//cylinderSource->SetCenter(0.0, 0.0, 0.0);
	sphereSource->SetRadius(r);
	sphereSource->SetPhiResolution(resolution);
	sphereSource->SetThetaResolution(resolution);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(sphereSource->GetOutputPort());
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(red, green, blue);
}

void Body::CreateCone(
	double r,
	double h,
	int resolution,
	double red,
	double green,
	double blue)
{
	// Create a cone
	vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
	//cylinderSource->SetCenter(0.0, 0.0, 0.0);
	coneSource->SetRadius(r);
	coneSource->SetHeight(h);
	coneSource->SetResolution(resolution);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(coneSource->GetOutputPort());
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(red, green, blue);
}

void Body::CreateLine(
	double *P1,
	double *P2,
	double red,
	double green,
	double blue)
{
	// Create line
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
	pts->InsertNextPoint(P1);
	pts->InsertNextPoint(P2);
	polyData->SetPoints(pts);
	vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
	line->GetPointIds()->SetId(0, 0);
	line->GetPointIds()->SetId(1, 1);
	lines->InsertNextCell(line);
	polyData->SetLines(lines);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(red, green, blue);
}

void Body::CreateDashedLine(
	double *P1,
	double *P2,
	double onLength,
	double offLength,
	double red,
	double green,
	double blue)
{
	// Create lines
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

	double patternLength = onLength + offLength;

	double dP[3];

	RVLDIF3VECTORS(P2, P1, dP);

	double length = sqrt(RVLDOTPRODUCT3(dP, dP));

	double V[3];

	RVLSCALE3VECTOR2(dP, length, V);

	double currentLength = 0.0;

	int i = 0;

	double POn[3], POff[3];
	double s;

	while (currentLength < length)
	{
		RVLSCALE3VECTOR(V, currentLength, dP);

		RVLSUM3VECTORS(P1, dP, POn);

		s = currentLength + onLength;

		if (s > length)
			s = length;

		RVLSCALE3VECTOR(V, s, dP);

		RVLSUM3VECTORS(P1, dP, POff);

		pts->InsertNextPoint(POn);
		pts->InsertNextPoint(POff);

		currentLength += patternLength;

		i++;
	}

	int nLines = i;

	polyData->SetPoints(pts);

	vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

	vtkSmartPointer<vtkLine> line;

	for (i = 0; i < nLines; i++)
	{
		line = vtkSmartPointer<vtkLine>::New();

		line->GetPointIds()->SetId(0, 2 * i);
		line->GetPointIds()->SetId(1, 2 * i + 1);

		lines->InsertNextCell(line);
	}

	polyData->SetLines(lines);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(red, green, blue);
}

void Body::CreateText(
	char *text,
	double red,
	double green,
	double blue)
{
	// Create text
	vtkSmartPointer<vtkVectorText> textSource = vtkSmartPointer<vtkVectorText>::New();
	textSource->SetText(text);
	textSource->Update();

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper =	vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(textSource->GetOutputPort());
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(red, green, blue);
}

void Body::SetCaption(
	char *text,
	double *P,
	double *offset,
	double red,
	double green,
	double blue,
	double fontSize)
{
	caption = vtkSmartPointer<vtkCaptionActor2D>::New();

	caption->SetCaption(text);
	caption->BorderOff();
	caption->LeaderOff();
	caption->SetPosition(offset);
	caption->GetCaptionTextProperty()->SetColor(red, green, blue);
	caption->GetCaptionTextProperty()->SetFontFamilyToTimes();
	//caption->SetWidth(0.07);
	caption->SetHeight(fontSize);

	captionAttachmentPoint[0] = P[0];
	captionAttachmentPoint[1] = P[1];
	captionAttachmentPoint[2] = P[2];
}

void Body::AddPart(Body * pPart)
{
	parts.push_back(pPart);
}


void Body::Translate(double x, double y, double z)
{
	pose->Translate(x, y, z);

	vtkMatrix4x4 *debugM = pose->GetMatrix();

	//double *t_ = pose->GetPosition();

	//int debug = 0;
}

void Body::Transform(double *T)
{
	pose->SetMatrix(T);

	vtkMatrix4x4 *debugM = pose->GetMatrix();
}


void Body::RotX(double q)
{
	pose->RotateX(q);

	//double *q_ = pose->GetOrientationWXYZ();
}

void Body::RotY(double q)
{
	pose->RotateY(q);
}

void Body::RotZ(double q)
{
	pose->RotateZ(q);
}


void Body::AddToRenderer(
	vtkSmartPointer<vtkRenderer> renderer,
	vtkSmartPointer<vtkTransform> TIn)
{
	vtkSmartPointer<vtkTransform> T;

	if (TIn)
	{
		T = TIn;

		//actor->SetUserTransform(T);

		//vtkMatrix4x4 *debugM = actor->GetUserTransform()->GetMatrix();

		//double *t = actor->GetPosition();

		//vtkMatrix4x4 *debugM = T->GetMatrix();

		if (actor)
		{
			double *t_ = T->GetPosition();

			actor->SetPosition(t_);

			double *q_ = T->GetOrientationWXYZ();

			actor->RotateWXYZ(q_[0], q_[1], q_[2], q_[3]);
		}
		
		//int debug = 0;
	}
	else
	{
		T = vtkSmartPointer<vtkTransform>::New();

		T->Identity();
	}

	if (actor)
		renderer->AddActor(actor);

	if (caption)
	{
		double P[3];

		T->TransformPoint(captionAttachmentPoint, P);

		caption->SetAttachmentPoint(P);

		renderer->AddActor(caption);
	}

	//vtkMatrix4x4 *debugM = T->GetMatrix();		

	vtkSmartPointer<vtkTransform> T_ = vtkSmartPointer<vtkTransform>::New();

	for (Body *part : parts)
	{
		T_->Identity();

		T_->Concatenate(T);

		//debugM = T_->GetMatrix();

		//debugM = part->pose->GetMatrix();

		T_->Concatenate(part->pose);

		//debugM = T_->GetMatrix();

		part->AddToRenderer(renderer, T_);
	}
}

void Body::AddToDisplay(
	Display *pWindow,
	vtkSmartPointer<vtkTransform> T)
{
	AddToRenderer(pWindow->GetRenderer());
}

void Body::SetOpacity(double opacity)
{
	actor->GetProperty()->SetOpacity(opacity);
}

void Body::SetCaptionOffset(double x, double y)
{
	caption->SetPosition(x, y);
}

void Body::SetLineWidth(double width)
{
	actor->GetProperty()->SetLineWidth(width);
}

void Body::LoadFromFile(
	char *fileName,
	bool bComputeNormals)
{
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(fileName);
	reader->Update();
	vtkSmartPointer<vtkPolyData> pPolygonData = reader->GetOutput();

	// calculate normals

	if (bComputeNormals)
	{
		vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
		normalGenerator->SetInputData(pPolygonData);
		normalGenerator->ComputePointNormalsOn();
		normalGenerator->ComputeCellNormalsOn();
		normalGenerator->Update();

		pPolygonData = normalGenerator->GetOutput();
	}

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(pPolygonData);
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	//actor->GetProperty()->SetColor(red, green, blue);

	//actor->GetProperty()->SetInterpolationToPhong();

	//int interpolationMethod = actor->GetProperty()->GetInterpolation();
}