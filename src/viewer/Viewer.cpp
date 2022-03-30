#include "Viewer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <vtkCamera.h>
#include <vtkTransform.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

// Viewer
Viewer::Viewer(unsigned int _width, unsigned int _height)
	: width(_width), height(_height), cloud(new pcl::PointCloud<pcl::PointXYZRGB>) {
	window = vtkSmartPointer<vtkRenderWindow>::New();
	interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    style = vtkSmartPointer<MyInteractor>::New();
    keyboard_callback = vtkSmartPointer<KeyboardCallback>::New();
}

Viewer::Viewer() 
	: Viewer(0, 0) {
}

void Viewer::initViewer(const std::string& cloudPath) {
	// Point cloud
	pcl::io::loadPCDFile(cloudPath, *cloud);

	vtkSmartPointer<vtkPolyData> cloud_data = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::pointCloudTovtkPolyData(*cloud, cloud_data);

	// Window
	window->SetSize(width, height);

    // Visualize
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(cloud_data);

    vtkNew<vtkNamedColors> colors;
    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(1);

    renderer->AddActor(actor);
    renderer->SetBackground(colors->GetColor3d("Black").GetData());
    renderer->GradientBackgroundOn();
    renderer->SetViewport(0.0, 0.0, 1.0, 1.0);

    window->AddRenderer(renderer);
    renderer->ResetCamera();
    interactor->SetRenderWindow(window);

    // Add axes
    vtkNew<vtkAxesActor> axes;
    vtkNew<vtkOrientationMarkerWidget> widget;
    double rgba[4]{ 0.0, 0.0, 0.0, 0.0 };
    colors->GetColor("Carrot", rgba);
    widget->SetOutlineColor(rgba[0], rgba[1], rgba[2]);
    widget->SetOrientationMarker(axes);
    widget->SetInteractor(interactor);
    widget->SetViewport(0.0, 0.0, 0.2, 0.2);
    widget->SetEnabled(1);
    widget->InteractiveOn();

    interactor->Initialize();
    interactor->CreateRepeatingTimer(100);

    // Render once
    window->Render();

    // Mouse Interactorf
    style->SetAutoAdjustCameraClippingRange(true);
    interactor->SetInteractorStyle(style);

    // Keyboard callback
    interactor->AddObserver(vtkCommand::KeyPressEvent, keyboard_callback);

    camera = renderer->GetActiveCamera();

    interactor->Start();
}