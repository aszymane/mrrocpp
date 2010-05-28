/*
 * image_switching_types.h
 *
 *  Created on: May 12, 2010
 *      Author: aszymane
 */

#ifndef IMAGE_SWITCHING_TYPES_H_
#define IMAGE_SWITCHING_TYPES_H_

namespace image_switching_types{

struct position_based_position
{
	/** Translation along X axis, from tpoint located in front of the camera. */
	double x;
	/** Translation along Y axis, from point located in front of the camera. */
	double y;
	/** Translation along Z axis, distance from point located in front of the camera. */
	double z;

	/** Rotation along Z axis. */
//	double gamma;
};


struct position_based_reading
{
	/** Set to true only if object is found in the image. */
	bool tracking;
	/** Set to true only if object is near the grabber. */
	bool found;
	/** Error calculated by FraDIA task and passed to MRROC++ ECP generator. */
	position_based_position error;
};

struct image_based_dimensions
{
	double center_x;
	double center_y;
	double diameter;
};

struct fradia_configuration
{
	bool search;
};

}

#endif /* IMAGE_SWITCHING_TYPES_H_ */
