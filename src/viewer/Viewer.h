#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <functional>

#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/common/common.h>

#include <vtkActor.h>
#include <vtkAppendFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkAutoInit.h>
#include <vtkCamera.h>
#include <vtkCameraActor.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCleanPolyData.h>
#include <vtkCommand.h>
#include <vtkDataReader.h>
#include <vtkDataSetMapper.h>
#include <vtkEDLShading.h>
#include <vtkInformation.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkParallelCoordinatesInteractorStyle.h>
#include <vtkPropPicker.h>
#include <vtkPLYReader.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyLine.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSimplePointsReader.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkWorldPointPicker.h>

#include "../geometry/Vec.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#define MouseCallback std::function<void(int mouseX, int mouseY)>
#define KeyCallback std::function<void(const std::string& key, bool shiftDown)>

class Viewer {
public:
    // Mouse events
    class MyInteractor : public vtkInteractorStyleTrackballCamera {
    private:
        MouseCallback leftButtonCallback;
        MouseCallback rightButtonCallback;
    public:
        vtkTypeMacro(MyInteractor, vtkInteractorStyleTrackballCamera);

        static MyInteractor* New() {
            return new MyInteractor;
        }
    public:
        virtual void OnLeftButtonDown() override {
            int mouseX, mouseY;
            this->Interactor->GetEventPosition(mouseX, mouseY);
            leftButtonCallback(mouseX, mouseY);
            vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
        }
        virtual void OnRightButtonDown() override {
            int mouseX, mouseY;
            this->Interactor->GetEventPosition(mouseX, mouseY);
            rightButtonCallback(mouseX, mouseY);
            vtkInteractorStyleTrackballCamera::OnRightButtonDown();
        }
    public:
        inline void setLeftButtonDownCallback(const MouseCallback& leftButtonCallback) {
            this->leftButtonCallback = leftButtonCallback;
        }

        inline void setRightButtonDownCallback(const MouseCallback& rightButtonCallback) {
            this->rightButtonCallback = rightButtonCallback;
        }
    };

    // Keyboard events
    class KeyboardCallback : public vtkCommand {
    private:
        KeyCallback keyCallback;
    public:
        vtkTypeMacro(KeyboardCallback, vtkCommand);

        static KeyboardCallback* New() {
            return new KeyboardCallback;
        }

        void Execute(vtkObject* caller, unsigned long vtkNotUsed(eventId), void* vtkNotUsed(callData)) override {
            vtkRenderWindowInteractor* interactor = vtkRenderWindowInteractor::SafeDownCast(caller);
            std::string key(interactor->GetKeySym());
            bool shiftDown = interactor->GetShiftKey();
            keyCallback(key, shiftDown);
        }
    public:
        inline void setKeyPressedCallback(const KeyCallback& keyCallback) {
            this->keyCallback = keyCallback;
        }
    };
private:
    unsigned int width, height;

    // Cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    // Rendering
    vtkSmartPointer<vtkRenderWindow> window;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;
    vtkNew<vtkRenderer> renderer;
    vtkSmartPointer<vtkCamera> camera;

    // Callbacks
    vtkSmartPointer<MyInteractor> style;
    vtkSmartPointer<KeyboardCallback> keyboard_callback;
public:
    Viewer(unsigned int _width, unsigned int _height);
    Viewer();
    ~Viewer() = default;
public:
    void initViewer(const std::string& cloudPath);
public:
    inline void setLeftButtonDownCallback(const MouseCallback& leftButtonCallback) {
        style->setLeftButtonDownCallback(leftButtonCallback);
    }

    inline void setRightButtonDownCallback(const MouseCallback& rightButtonCallback) {
        style->setRightButtonDownCallback(rightButtonCallback);
    }

    inline void setKeyPressedCallback(const KeyCallback& keyCallback) {
        keyboard_callback->setKeyPressedCallback(keyCallback);
    }
public:
    inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr& getCloud() {
        return cloud;
    }

    inline vtkSmartPointer<vtkRenderWindow>& getWindow() {
        return window;
    }

    inline vtkSmartPointer<vtkRenderWindowInteractor>& getInteractor() {
        return interactor;
    }

    inline vtkNew<vtkRenderer>& getRenderer() {
        return renderer;
    }

    inline vtkSmartPointer<vtkCamera>& getCamera() {
        return camera;
    }
};