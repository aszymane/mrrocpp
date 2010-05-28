/*
 * VisualServoTypes.h
 *
 *  Created on: Apr 7, 2010
 *      Author: mboryn
 */

#ifndef VISUALSERVOTYPES_H_
#define VISUALSERVOTYPES_H_

namespace visual_servo_types {

/**
 * Structure for object position for image based visual servo.
 * Structure must be identical in MRROC++ and FraDIA.
 * This structure is sent from FraDIA to MRROC++.
 */
struct image_based_position
{
	/** Translation along X axis, from the center of the image. */
	int x;
	/** Translation along Y axis, from the center of the image. */
	int y;
	/** Translation along Z axis, distance from point located in front of the camera. */
	int z;

	/** Rotation along Z axis. */
	double gamma;
};

/** Camera parameters retrieved by cvCalibrateCamera2() from OpenCV. */
struct distortion_correction_parameters
{
	/** Camera intrinsics matrix */
	double intrinsics[3][3];
	/** Distortion correction coefficiens:  k1, k2, p1,p2, k3 */
	double distortion[5];
};

/** Configuration of position based visual servo. This structure is sent from MRROC++ to FraDIA. */
struct position_based_configuration
{
	distortion_correction_parameters dcp;
};

/** Configuration of image based visual servo. This structure is sent from MRROC++ to FraDIA. */
struct image_based_configuration
{
	/** Set to true if dcp and desired_position are valid */
	bool set_parameters;

	distortion_correction_parameters dcp;
	/** parameters for calculating camera-object distance: Z = A / (diameter + B) + C. */
	double z_estimation_A;
	double z_estimation_B;
	double z_estimation_C;

	/** Object's desired position in image. Reference point for calculating error. */
	image_based_position desired_position;
};

struct image_based_reading
{
	/** Set to true only if object is found in the image. */
	bool tracking;
	/** Error calculated by FraDIA task and passed to MRROC++ ECP generator. */
	image_based_position error;
};

struct position_based_reading
{
	/** Set to true only if object is found in the image. */
	bool tracking;

	/** Homogeneous matrix with object position with respect to the camera. */
	double position[3][4];
};

} //namespace visual_servo_types

#endif /* VISUALSERVOTYPES_H_ */
