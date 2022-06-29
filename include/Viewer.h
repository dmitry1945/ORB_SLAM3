


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM3
{

	class System;
	class FrameDrawer;
	class Settings;
	class Tracking;

	class Viewer
	{
	public:
		Viewer();

		virtual void Init(System* pSystem, FrameDrawer* pFrameDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings);
		
		virtual void newParameterLoader(Settings* settings);
		
		// Main thread function. Draw points, keyframes, the current camera pose and the last processed
		// frame. Drawing is refreshed according to the camera fps. We use Pangolin.
		
		virtual void Run();
		
		virtual void RunThread();

		virtual void RequestFinish();

		virtual void RequestStop();

		virtual bool isFinished();

		virtual bool isStopped();

		virtual bool isStepByStep();

		virtual void Release();

		virtual void SetBoth(bool both);
	};

}


#endif // VIEWER_H


