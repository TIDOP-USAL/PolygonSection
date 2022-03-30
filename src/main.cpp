#include <iostream>

#include "viewer/Viewer.h"

vtkSmartPointer<vtkActor> actorPolygon = vtkSmartPointer<vtkActor>::New();
std::vector<Vec3d> polygonVertices;
bool section = false;

void leftButton(Viewer& viewer, int mouseX, int mouseY) {

    if (section) {
        if (viewer.getCamera() == nullptr) return;

        //------------------ TRANSFORM MOUSE POSITION ----------------------
        double selectionX = mouseX, selectionY = mouseY, selectionZ = 0;

        // Convert to world space
        double cameraFP[4];
        double display[3], * world;
        double* displayCoord;

        // Get camera focal point and position. Convert to display (screen)
        // coordinates
        viewer.getCamera()->GetFocalPoint(cameraFP);
        cameraFP[3] = 1.0;

        viewer.getRenderer()->SetWorldPoint(cameraFP[0], cameraFP[1], cameraFP[2], cameraFP[3]);
        viewer.getRenderer()->WorldToDisplay();
        displayCoord = viewer.getRenderer()->GetDisplayPoint();
        selectionZ = displayCoord[2];

        // now convert the display point to world coordinates
        display[0] = selectionX;
        display[1] = selectionY;
        display[2] = selectionZ;

        viewer.getRenderer()->SetDisplayPoint(display);
        viewer.getRenderer()->DisplayToWorld();
        world = viewer.getRenderer()->GetWorldPoint();

        // Add world point to polygon
        polygonVertices.push_back(Vec3d(world[0] / world[3], world[1] / world[3], world[2] / world[3]));
        //----------------------------------------


        //------------------ DRAW ----------------------
        vtkNew<vtkPoints> pointsPolygon;
        for (int i = 0; i < polygonVertices.size(); i++) {
            pointsPolygon->InsertNextPoint(polygonVertices[i].x, polygonVertices[i].y, polygonVertices[i].z);
        }
        pointsPolygon->InsertNextPoint(polygonVertices[0].x, polygonVertices[0].y, polygonVertices[0].z);

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

        viewer.getRenderer()->AddActor(actorPolygon);
    }
}

void rightButton(Viewer& viewer, int mouseX, int mouseY) {
    if (section) {
        section = false;
        polygonVertices.clear();
        viewer.getRenderer()->RemoveActor(actorPolygon);
    }
}

void keyBoard(Viewer& viewer, const std::string& key) {
    if (key == "s" || key == "S")
        section = !section;
    std::cout << "Section " << section << std::endl;
}

int main() {

    // Set up viewer
    Viewer viewer(800, 600);

    // Callbacks
    viewer.setLeftButtonDownCallback([&](int mouseX, int mouseY) { leftButton(viewer, mouseX, mouseY); });
    viewer.setRightButtonDownCallback([&](int mouseX, int mouseY) { rightButton(viewer, mouseX, mouseY); });
    viewer.setKeyPressedCallback([&](const std::string& key, bool shiftDown) {  keyBoard(viewer, key); });

    // Start rendering
    viewer.initViewer("femur.pcd");

    return 0;
}