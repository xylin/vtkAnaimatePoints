#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkGlyph3D.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProgrammableFilter.h>
#include <vtkCallbackCommand.h>
#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkProperty.h>

#include <vrpn_Tracker.h>
#include <quat.h>

#include <iostream>

static void VRPN_CALLBACK tracker_handler(void *userdata, const vrpn_TRACKERCB t);

#define iSKELETON_NUM 24

// Position and orientation of each joint

static double position[iSKELETON_NUM][3];
static double orientation[iSKELETON_NUM][4];

void TimerCallbackFunction ( vtkObject* caller, long unsigned int eventId, void* clientData, void* callData );

// Globals
unsigned int counter2 = 0;

vrpn_Tracker_Remote *remote;


void AdjustPoints2(void* arguments)
{
	// Clear console
	system("cls");

	// Update the joint positions
	remote->mainloop();				

	std::cout << "AdjustPoints2" << std::endl;

	vtkProgrammableFilter* programmableFilter = static_cast<vtkProgrammableFilter*>(arguments);
		
	vtkPolyData* ppoly=programmableFilter->GetPolyDataInput();
	vtkPoints* inPts = ppoly->GetPoints();
	vtkIdType numPts = inPts->GetNumberOfPoints();

	vtkSmartPointer<vtkPoints> newPts = vtkSmartPointer<vtkPoints>::New();

	newPts->SetNumberOfPoints(numPts);

	cout << numPts << endl;
		
	double eulerOrientation[3]={0,0,0};

	for(vtkIdType i = 0; i < numPts; i++)
	{
		double p[3];
		inPts->GetPoint(i, p);
		
		p[0] = position[i][0];
		p[1] = position[i][1];
		p[2] = position[i][2];
		
		newPts->SetPoint(i, p);
		
		if(i<3)
		{
			// Convert orientation quaternion to euler angles
			q_to_euler(eulerOrientation, orientation[i]);

			// Output updated data
			printf("\nJoint: %i\n", i);
			printf("Position: \n X: %f\n Y: %f\n Z: %f\n", p[0], p[1], p[2]);			
			printf("Orientation: \n Yaw: %f\n Pitch: %f\n Roll: %f\n", eulerOrientation[0], eulerOrientation[1], eulerOrientation[2]);
		}
	}

	programmableFilter->GetPolyDataOutput()->CopyStructure(programmableFilter->GetPolyDataInput());
	programmableFilter->GetPolyDataOutput()->SetPoints(newPts);	
}

int main(int, char *[])
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();	

	for(int i = 0; i < iSKELETON_NUM; i++)
	{
		for(int j = 0; j < 3; j++)
			position[i][j] =0;

		for(int j = 0; j < 4; j++)
			orientation[i][j] =0;		

		points->InsertNextPoint(position[i][0], position[i][1], position[i][2]);
	}

		// Create a remote to track output
	remote = new vrpn_Tracker_Remote("Tracker0@localhost");
	remote->register_change_handler(NULL, tracker_handler);
	remote->shutup = true;
	
	vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();

#if VTK_MAJOR_VERSION <= 5
	vertexFilter->SetInputConnection(pointsPolydata->GetProducerPort());
#else
	vertexFilter->SetInputData(pointsPolydata);
#endif
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

	polydata->ShallowCopy(vertexFilter->GetOutput());

	// Setup colors
	unsigned char red[3] = {255, 0, 0};
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);
	colors->SetName ("Colors");

	for(int i = 0; i < iSKELETON_NUM; i++)
	{
		colors->InsertNextTupleValue(red);
	}
	 
	polydata->GetPointData()->SetScalars(colors);	
	
	vtkSmartPointer<vtkProgrammableFilter> programmableFilter = vtkSmartPointer<vtkProgrammableFilter>::New();

	programmableFilter->SetInput(polydata);

	programmableFilter->SetExecuteMethod(AdjustPoints2, programmableFilter);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputConnection(programmableFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(10);

	// Create a renderer, render window, and interactor
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

	renderWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renderWindowInteractor->SetRenderWindow(renderWindow);

	// Initialize must be called prior to creating timer events.
	renderWindowInteractor->Initialize();
	renderWindowInteractor->CreateRepeatingTimer(200);

	vtkSmartPointer<vtkCallbackCommand> timerCallback = vtkSmartPointer<vtkCallbackCommand>::New();

	timerCallback->SetCallback ( TimerCallbackFunction );
	timerCallback->SetClientData(programmableFilter);

	renderWindowInteractor->AddObserver ( vtkCommand::TimerEvent, timerCallback );

	// Add the actor to the scene
	renderer->AddActor(actor);
	renderer->SetBackground(0,255,0); // Background color white

	// Render and interact
	renderWindow->Render();
	renderWindowInteractor->Start();
	
	return EXIT_SUCCESS;
}


void TimerCallbackFunction ( vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* clientData, void* vtkNotUsed(callData) )
{
	cout << "timer callback" << endl;

	vtkSmartPointer<vtkProgrammableFilter> programmableFilter = 
		static_cast<vtkProgrammableFilter*>(clientData);

	vtkRenderWindowInteractor *iren = 
		static_cast<vtkRenderWindowInteractor*>(caller);

	programmableFilter->Modified();

	iren->Render();

	counter2++;

	if(counter2>50)
		counter2=0;

}

static void VRPN_CALLBACK tracker_handler(void *userdata, const vrpn_TRACKERCB t)
{
        // Store the updated position and orientation of the targeted joint
        q_vec_copy(position[t.sensor], t.pos);
        q_copy(orientation[t.sensor], t.quat);


//	std::cout << "Tracker '" << t.sensor << "' : " << t.pos[0] << "," <<  t.pos[1] << "," << t.pos[2] << std::endl;
}