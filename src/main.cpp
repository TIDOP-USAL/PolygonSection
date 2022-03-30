#include <iostream>
#include <vector>

#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/common/common.h>

#include <vtkActor.h>
#include <vtkActor2DCollection.h>
#include <vtkActorCollection.h>
#include <vtkAppendFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkAutoInit.h>
#include <vtkCamera.h>
#include <vtkCameraActor.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCleanPolyData.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkDataReader.h>
#include <vtkDataSetMapper.h>
#include <vtkEDLShading.h>
#include <vtkHull.h>
#include <vtkInformation.h>
#include <vtkInformationStringKey.h>
#include <vtkInteractorStyleRubberBand3D.h>
#include <vtkInteractorStyleRubberBandZoom.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLODActor.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkMutexLock.h>
#include <vtkNamedColors.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkParallelCoordinatesInteractorStyle.h>
#include <vtkPropPicker.h>
#include <vtkPLYReader.h>
#include <vtkPointPicker.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyLine.h>
#include <vtkProperty.h>
#include <vtkRectilinearGrid.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkRenderStepsPass.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSimplePointsReader.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkSTLReader.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkUnsignedCharArray.h>
#include <vtkWorldPointPicker.h>

#include <vtkTransform.h>
#include <vtkCamera.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
vtkSmartPointer<vtkActor> actorPolygon = vtkSmartPointer<vtkActor>::New();
std::vector<vtkVector3f> polygonVertices;

vtkNew<vtkRenderer> renderer;
vtkSmartPointer<vtkCamera> camera = nullptr;
bool section = false;

// Mouse events
class MyInteractor : public vtkInteractorStyleTrackballCamera {
public:
    static MyInteractor* New();
    vtkTypeMacro(MyInteractor, vtkInteractorStyleTrackballCamera);
public:
    virtual void OnLeftButtonDown() {

        int mouseX, mouseY;
        this->Interactor->GetEventPosition(mouseX, mouseY);
        std::cout << "Mouse x: " << mouseX << ", Mouse y: " << mouseY << std::endl;

        if (section) {
            
            // 2D vertex
            vtkVector3f vertex(mouseX, mouseY, 0);
            double xyz[3] = {mouseX, mouseY, 0};

            // Transform vertex
            if (camera == nullptr) return;
           
            /*
            // WARNING: vtkWorldPointPicker uses Depth buffer for picking, it makes picking weird for rendered pixels, IT IS DISABLED IN THE CODE BELOW (-TRANSFORM MOUSE POSITION-)
            vtkSmartPointer<vtkWorldPointPicker> worldPointPicker = vtkSmartPointer<vtkWorldPointPicker>::New();
            worldPointPicker->Pick(xyz, renderer);
            double* worldPosition = worldPointPicker->GetPickPosition();

            polygonVertices.push_back(vtkVector3f(worldPosition[0], worldPosition[1], worldPosition[2]));
            */


            //------------------ TRANSFORM MOUSE POSITION ----------------------
            double selectionX = mouseX, selectionY = mouseY, selectionZ = 0;

            // Convert to world space
            double cameraFP[4];
            double display[3], * world;
            double* displayCoord;

            // Get camera focal point and position. Convert to display (screen)
            // coordinates
            camera = renderer->GetActiveCamera();
            camera->GetFocalPoint(cameraFP);
            cameraFP[3] = 1.0;

            renderer->SetWorldPoint(cameraFP[0], cameraFP[1], cameraFP[2], cameraFP[3]);
            renderer->WorldToDisplay();
            displayCoord = renderer->GetDisplayPoint();
            selectionZ = displayCoord[2];
            vtkDebugMacro(<< "computed z from focal point: " << selectionZ);

            // now convert the display point to world coordinates
            display[0] = selectionX;
            display[1] = selectionY;
            display[2] = selectionZ;

            renderer->SetDisplayPoint(display);
            renderer->DisplayToWorld();
            world = renderer->GetWorldPoint();

            // Add world point to polygon
            polygonVertices.push_back(vtkVector3f(world[0] / world[3], world[1] / world[3], world[2] / world[3]));
            //----------------------------------------

            vtkNew<vtkPoints> pointsPolygon;
            for (int i = 0; i < polygonVertices.size(); i++) {
                pointsPolygon->InsertNextPoint(polygonVertices[i].GetX(), polygonVertices[i].GetY(), polygonVertices[i].GetZ());
            }
            pointsPolygon->InsertNextPoint(polygonVertices[0].GetX(), polygonVertices[0].GetY(), polygonVertices[0].GetZ());

            vtkNew<vtkPolyLine> polyLinePolygon;
            polyLinePolygon->GetPointIds()->SetNumberOfIds(polygonVertices.size() + 1);

            for (unsigned int i = 0; i < polygonVertices.size() + 1; i++)
                polyLinePolygon->GetPointIds()->SetId(i, i);

            vtkNew<vtkCellArray> cellsPolygon;
            cellsPolygon->InsertNextCell(polyLinePolygon);

            // Create a polydata to store everything in
            vtkNew<vtkPolyData> polyDataPolygon;

            // Add the points to the dataset
            polyDataPolygon->SetPoints(pointsPolygon);

            // Add the lines to the dataset
            polyDataPolygon->SetLines(cellsPolygon);

            // Setup actor and mapper
            vtkNew<vtkPolyDataMapper> mapperLinePolygon;
            mapperLinePolygon->SetInputData(polyDataPolygon);

            actorPolygon->SetMapper(mapperLinePolygon);
            actorPolygon->GetProperty()->SetColor(1.0, 0.0, 0.0);
            actorPolygon->GetProperty()->SetLineWidth(1);

            this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actorPolygon);
        }

        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    virtual void OnRightButtonDown() {
        if (section) {
            section = false;
            polygonVertices.clear();
            this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actorPolygon);
        }
        vtkInteractorStyleTrackballCamera::OnRightButtonDown();
    }
};
vtkStandardNewMacro(MyInteractor);

// Keyboard events
class KeyboardCallback : public vtkCommand {
public:
    vtkTypeMacro(KeyboardCallback, vtkCommand);

    static KeyboardCallback* New() {
        return new KeyboardCallback;
    }

    void Execute(vtkObject* caller, unsigned long vtkNotUsed(eventId), void* vtkNotUsed(callData)) {
        vtkRenderWindowInteractor* interactor = vtkRenderWindowInteractor::SafeDownCast(caller);

        vtkRenderer* renderer = interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();
        vtkRenderWindow* renderWindow = interactor->GetRenderWindow();

        vtkOpenGLRenderer* glrenderer = vtkOpenGLRenderer::SafeDownCast(renderer);
        vtkSmartPointer<vtkRenderStepsPass> basicPasses = vtkSmartPointer<vtkRenderStepsPass>::New();

        std::string key(interactor->GetKeySym());
        bool shift_down = interactor->GetShiftKey();

        if (key == "s" || key == "S") {
            section = !section;
            std::cout << "Section enabled: " << section << std::endl;
        };
    }
};

void initViewer() {

	// Point cloud
	pcl::io::loadPCDFile("femur.pcd", *cloud);

    vtkSmartPointer<vtkPolyData> cloud_data = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::pointCloudTovtkPolyData(*cloud, cloud_data);

    // Create window and interactor
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
    window->SetSize(800, 600);

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

    // Axes
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

    // Mouse Interactor
    vtkSmartPointer<MyInteractor> style = vtkSmartPointer<MyInteractor>::New();
    style->SetAutoAdjustCameraClippingRange(true);
    interactor->SetInteractorStyle(style);

    // Keyboard callback
    vtkSmartPointer<KeyboardCallback> keyboard_callback = vtkSmartPointer<KeyboardCallback>::New();
    interactor->AddObserver(vtkCommand::KeyPressEvent, keyboard_callback);

    camera = renderer->GetActiveCamera();

    interactor->Start();
}

int main() {

    initViewer();
	return 0;
}