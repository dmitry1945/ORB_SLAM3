#include "Viewer.h"

ORB_SLAM3::Viewer::Viewer()
{
}

void ORB_SLAM3::Viewer::Init(System * pSystem, FrameDrawer * pFrameDrawer, Tracking * pTracking, const string & strSettingPath, Settings* settings)
{
}

void ORB_SLAM3::Viewer::newParameterLoader(Settings* settings)
{
}

void ORB_SLAM3::Viewer::Run()
{
	this->RunThread();
}

void ORB_SLAM3::Viewer::RunThread()
{
}

void ORB_SLAM3::Viewer::RequestFinish()
{
}

void ORB_SLAM3::Viewer::RequestStop()
{
}

bool ORB_SLAM3::Viewer::isFinished()
{
	return false;
}

bool ORB_SLAM3::Viewer::isStopped()
{
	return false;
}

bool ORB_SLAM3::Viewer::isStepByStep()
{
	return false;
}

void ORB_SLAM3::Viewer::Release()
{
}
void ORB_SLAM3::Viewer::SetBoth(bool both)
{

}

