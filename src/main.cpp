#include <iostream>

#include "viewer/Viewer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include "geometry/Plane.h"
#include "geometry/Transform.h"
#include "geometry/Poly.h"

#define DRAW_PROJECTIONS

// Callbacks
void leftButton(Viewer& viewer, int mouseX, int mouseY);
void rightButton(Viewer& viewer, int mouseX, int mouseY);
void keyBoard(Viewer& viewer, const std::string& key);

// Algorithm
Vec3d toLocalSpace(Viewer& viewer, int mouseX, int mouseY);
void drawPolygon(Viewer& viewer);
void saveSection(Viewer& viewer, const std::vector<Vec3d>& polygon);

// Global vars
vtkSmartPointer<vtkActor> actorPolygon = vtkSmartPointer<vtkActor>::New();
std::vector<Vec3d> polygonVertices;
bool section = false;

#if defined(DRAW_PROJECTIONS)
std::vector<Vec3d> projectedPoints;
vtkSmartPointer<vtkActor> actorProjections = vtkSmartPointer<vtkActor>::New();
void drawProjections(Viewer& viewer);
#endif

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

Vec3d toLocalSpace(Viewer& viewer, int mouseX, int mouseY) {

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

    return Vec3d(world[0] / world[3], world[1] / world[3], world[2] / world[3]);
}

void drawPolygon(Viewer& viewer) {

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

#if defined(DRAW_PROJECTIONS)
void drawProjections(Viewer& viewer) {

    vtkNew<vtkPoints> points;
    for (int i = 0; i < projectedPoints.size(); i++) {
        points->InsertNextPoint(projectedPoints[i].x, projectedPoints[i].y, projectedPoints[i].z);
    }

    vtkNew<vtkPolyLine> polyLinePolygon;
    polyLinePolygon->GetPointIds()->SetNumberOfIds(projectedPoints.size());

    for (unsigned int i = 0; i < projectedPoints.size(); i++)
        polyLinePolygon->GetPointIds()->SetId(i, i);

    vtkNew<vtkCellArray> cellsPolygon;
    cellsPolygon->InsertNextCell(polyLinePolygon);

    // Create a polydata to store everything in
    vtkNew<vtkPolyData> polyDataPolygon;

    // Add the points to the dataset
    polyDataPolygon->SetPoints(points);

    // Add the lines to the dataset
    polyDataPolygon->SetLines(cellsPolygon);

    // Setup actor and mapper
    vtkNew<vtkPolyDataMapper> mapperLinePolygon;
    mapperLinePolygon->SetInputData(polyDataPolygon);

    actorProjections->SetMapper(mapperLinePolygon);
    actorProjections->GetProperty()->SetColor(0.0, 0.0, 1.0);
    actorProjections->GetProperty()->SetLineWidth(1);

    viewer.getRenderer()->AddActor(actorProjections);
}
#endif

void leftButton(Viewer& viewer, int mouseX, int mouseY) {

    if (section) {
        if (viewer.getCamera() == nullptr) return;
        // Add transformed vertex to polygon
        Vec3d localSpaceVertex = toLocalSpace(viewer, mouseX, mouseY);
        polygonVertices.push_back(localSpaceVertex);
        // Draw polygon
        drawPolygon(viewer);
    }
}

void rightButton(Viewer& viewer, int mouseX, int mouseY) {
    if (section) {
        // Save section
        saveSection(viewer, polygonVertices);
        // Draw projections
#if defined(DRAW_PROJECTIONS)
        drawProjections(viewer);
#endif
        // Reset polygon
        section = false;
        std::cout << "Section enabled: " << section << std::endl;
    }
}

void keyBoard(Viewer& viewer, const std::string& key) {
    if (key == "s" || key == "S")
        section = !section;
    if (section) {
        polygonVertices.clear();
#if defined(DRAW_PROJECTIONS)
        projectedPoints.clear();
        viewer.getRenderer()->RemoveActor(actorProjections);
#endif
        viewer.getRenderer()->RemoveActor(actorPolygon);
    }
    std::cout << "Section enabled: " << section << std::endl;
}

void saveSection(Viewer& viewer, const std::vector<Vec3d>& polygon) {
    
    // Section point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sectionCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Calculate plane which contains the polygon
    Vec3d firstPoint = polygon[0], secondPoint = polygon[1], thirdPoint = polygon[polygon.size() - 1];
    Plane plane = Plane::plane3points(firstPoint, secondPoint, thirdPoint);
    Vec3d normal = plane.getNormal();

    // Polygon
    Poly poly(polygon);

    // Trace lines for each point orthogonal to the plane and get its intersections
    for (pcl::PointXYZRGB& point : viewer.getCloud()->points) {

        // Line orthogonal to plane
        Line line = Line::fromPointVector(Vec3d(point.x, point.y, point.z), normal);

        // Calculate projection (intersection with plane)
        Vec3d projectedPoint = plane.lineIntersection(line);

        // If projected point is inside of the polygon, the point is inside of the section
        // As the z is always the same (the point and the polygon are in the same plane) we can ignore it
        if (poly.isPointInside(projectedPoint)) {
            sectionCloud->push_back(point);
#if defined(DRAW_PROJECTIONS)
            projectedPoints.push_back(projectedPoint);
#endif
        }
    }
    // Save section
    if (pcl::io::savePCDFileASCII("C:\\Users\\alber\\Desktop\\section.pcd", *sectionCloud) == -1)
        std::cout << "Couldn't save point cloud" << std::endl;
}