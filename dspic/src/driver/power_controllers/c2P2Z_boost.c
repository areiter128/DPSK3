/* ***************************************************************************************
 * z-Domain Control Loop Designer Version 0.9.0.61.
 * ***************************************************************************************
 * 2p2z compensation filter coefficients derived for following operating conditions:
 * ***************************************************************************************
 *
 * 	Controller Type:	2P2Z - Basic Current Mode Compensator
 * 	Sampling Frequency:	250000 Hz 
 * 	Fixed Point Format:	15
 * 	Scaling Mode:		3 - Dual Bit-Shift Scaling
 * 	Input Gain:			1
 * 
 * ***************************************************************************************/

#include "../h/driver/power_controllers/c2p2z_boost.h"

/* ***************************************************************************************
 * Data Arrays:
 * The cNPNZ_t data structure contains a pointer to derived coefficients in X-space and
 * other pointers to controller and error history in Y-space.
 * This source file declares the default parameters of the z-domain compensation filter.
 * These declarations are made publicly accessible through defines in c2p2z_boost.h
 * ***************************************************************************************/

	volatile C2P2Z_BOOST_CONTROL_LOOP_COEFFICIENTS_t __attribute__((space(xmemory), near)) c2p2z_boost_coefficients; // A/B-Coefficients 
	volatile uint16_t c2p2z_boost_ACoefficients_size = (sizeof(c2p2z_boost_coefficients.ACoefficients)/sizeof(c2p2z_boost_coefficients.ACoefficients[0])); // A-coefficient array size
	volatile uint16_t c2p2z_boost_BCoefficients_size = (sizeof(c2p2z_boost_coefficients.BCoefficients)/sizeof(c2p2z_boost_coefficients.BCoefficients[0])); // B-coefficient array size

	volatile C2P2Z_BOOST_CONTROL_LOOP_HISTORIES_t __attribute__((space(ymemory), far)) c2p2z_boost_histories; // Control/Error Histories 
	volatile uint16_t c2p2z_boost_ControlHistory_size = (sizeof(c2p2z_boost_histories.ControlHistory)/sizeof(c2p2z_boost_histories.ControlHistory[0])); // Control history array size
	volatile uint16_t c2p2z_boost_ErrorHistory_size = (sizeof(c2p2z_boost_histories.ErrorHistory)/sizeof(c2p2z_boost_histories.ErrorHistory[0])); // Error history array size

/* ***************************************************************************************
 * 	Pole&Zero Placement:
 * ***************************************************************************************
 *
 * 	fP0:	1200 Hz 
 * 	fP1:	30000 Hz 
 * 	fZ1:	400 Hz 
 *
 * ***************************************************************************************
 * 	Filter Coefficients and Parameters:
 * ***************************************************************************************/

	volatile fractional c2p2z_boost_ACoefficients [2] = 
	{
		0x5CF5,	// Coefficient A1 will be multiplied with controller output u(n-1)
		0xE30C	// Coefficient A2 will be multiplied with controller output u(n-2)
	};

	volatile fractional c2p2z_boost_BCoefficients [3] = 
	{
		0x69A9,	// Coefficient B0 will be multiplied with error input e(n)
		0x010F,	// Coefficient B1 will be multiplied with error input e(n-1)
		0x9766	// Coefficient B2 will be multiplied with error input e(n-2)
	};


	volatile int16_t c2p2z_boost_pre_scaler = 3;
	volatile int16_t c2p2z_boost_post_shift_A = -1;
	volatile int16_t c2p2z_boost_post_shift_B = 0;
	volatile fractional c2p2z_boost_post_scaler = 0x0000;

	volatile cNPNZ16b_t c2p2z_boost; // user-controller data object

/* ***************************************************************************************/

volatile uint16_t c2p2z_boost_Init(volatile cNPNZ16b_t* controller)
{
	volatile uint16_t i = 0;

	// Initialize controller data structure at runtime with pre-defined default values
	controller->status.value = CONTROLLER_STATUS_CLEAR;  // clear all status flag bits (will turn off execution))

	controller->ptrACoefficients = &c2p2z_boost_coefficients.ACoefficients[0]; // initialize pointer to A-coefficients array
	controller->ptrBCoefficients = &c2p2z_boost_coefficients.BCoefficients[0]; // initialize pointer to B-coefficients array
	controller->ptrControlHistory = &c2p2z_boost_histories.ControlHistory[0]; // initialize pointer to control history array
	controller->ptrErrorHistory = &c2p2z_boost_histories.ErrorHistory[0]; // initialize pointer to error history array
	controller->normPostShiftA = c2p2z_boost_post_shift_A; // initialize A-coefficients/single bit-shift scaler
	controller->normPostShiftB = c2p2z_boost_post_shift_B; // initialize B-coefficients/dual/post scale factor bit-shift scaler
	controller->normPostScaler = c2p2z_boost_post_scaler; // initialize control output value normalization scaling factor
	controller->normPreShift = c2p2z_boost_pre_scaler; // initialize A-coefficients/single bit-shift scaler

	controller->ACoefficientsArraySize = c2p2z_boost_ACoefficients_size; // initialize A-coefficients array size
	controller->BCoefficientsArraySize = c2p2z_boost_BCoefficients_size; // initialize A-coefficients array size
	controller->ControlHistoryArraySize = c2p2z_boost_ControlHistory_size; // initialize control history array size
	controller->ErrorHistoryArraySize = c2p2z_boost_ErrorHistory_size; // initialize error history array size


	// Load default set of A-coefficients from user RAM into X-Space controller A-array
	for(i=0; i<controller->ACoefficientsArraySize; i++)
	{
		c2p2z_boost_coefficients.ACoefficients[i] = c2p2z_boost_ACoefficients[i];
	}

	// Load default set of B-coefficients from user RAM into X-Space controller B-array
	for(i=0; i<controller->BCoefficientsArraySize; i++)
	{
		c2p2z_boost_coefficients.BCoefficients[i] = c2p2z_boost_BCoefficients[i];
	}

	// Clear error and control histories of the 2P2Z controller
	c2p2z_boost_Reset(&c2p2z_boost);

	return(1);
}


