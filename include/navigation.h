#pragma once


void offsetCoordinates(float offset, float x, float y, float phi, float &offsetX, float&offsetY);

class Navigation {
	public:
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
		
};