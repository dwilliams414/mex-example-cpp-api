/*EXAMPLE: MEX Integration of CR3BP Equations of Motion
* This is an example script illustrating how to generate a mex function for integrating the CR3BP
* equations of motion (not variational equations) using BOOST integrators (RK78) and the C++ Mex API
* 
* Function Call (MATLAB):
* [t, x] = mexCR3BP_Example(x0, t_span, mu)
* [t, x] = mexCR3BP_Example(x0, t_span, mu, abs_tol, rel_tol, init_step)
* 
* Inputs (Required):
*	x0: 6x1 initial state vector
*	t_span: nx1 (or 1xn), where n>=2.  
*			If n = 2, an adaptive step is used in integration and results of each step are returned
*			If n >=2, the solution is evaluated ONLY at the times specified in t_span
*	mu: Mass Parameter
* Inputs (Optional):
*	abs_tol: Absolute tolerance for use in integration (default 1e-16)
*	rel_tol: Relative tolerance for use in integration (default 1e-13)
*	init_step: Initial step size for integration (default 1e-10)
*/

// Include Necessary Header Files
#pragma warning(push)
#pragma warning(disable : 4996) // Including Boost throws a warning due to use of sprintf.  This ignores that warning

#include "mex.hpp" // For MEX interface and functions
#include "mexAdapter.hpp" // For MEX interface and functions
#include "boost/numeric/odeint.hpp" // For odeint integration utilities
#include <math.h> // For pow and sqrt

#pragma warning(pop) // Turn all subsequent warnings back on

// Define Namespaces for Convenience
using matlab::mex::ArgumentList; // For Inputs & Outputs to MEX function
using namespace matlab::data;  // For ArrayFactory and MATLAB defined Array types in C++ API
using namespace boost::numeric; // For odeint & steppers

// Define C++ Standard Type to represent states
typedef std::vector<double> state_type;

// Define BOOST's 7-8 Stepper as rk78
typedef odeint::runge_kutta_fehlberg78<state_type> rk78;

/************************************* Declare Classes ***************************************/
/*MexFunction: class definition required to interface with MATLAB and allow calling of C++ Code
* This class (with this name) MUST exist somewhere and inherit from matlab::mex::Function.
* It is how MATLAB interfaces with your C++ Code.  Its operator() method is what is called when you call your MEX function
* from MATLAB
*/
class MexFunction : public matlab::mex::Function
{
public:
	/*Declare Methods*/
	// operator(): Must take two argument lists corresponding to outputs and inputs with MATLAB.  This is where
	// we write the code that our function will execute when called.
	void operator()(ArgumentList outputs, ArgumentList inputs);

	// Argument Validation function: Throw errors if inputs to MEX function are bad
	void validate_args(ArgumentList outputs, ArgumentList inputs);
};

/*CR3BP: Class definition used as container for system mass parameter, equations of motion, and other CR3BP-type stuff.
* You can use this as (one possible) template for your own code and expand upon it as necessary.
*/
class CR3BP
{
public:
	// Member Variables
	double m_mu; // System Mass Parameter

	// Constructor Declaration
	CR3BP(double mu);

	// Declare Operator for Class (Direct Call to EOM's).  This allows us to 
	// easily pass the mass parameter when we integrate, since we can just pass
	// the CR3BP object.
	void operator()(const state_type& state, state_type& state_dot, const double t);

	// CR3BP EOM's, No Variational Equations
	void eom6(const state_type& state, state_type& state_dot, const double t);

	// CR3BP EOM's, w/ Variational Equations (1st Order)
	void eom42(const state_type& state, state_type& state_dot, const double t);

	// Evaluate Distance r to second primary
	double eval_r(const state_type& state);

	// Evaulate the Distance d from the first primary (larger)
	double eval_d(const state_type& state);

	// Evaluate uxx pseudopotential term
	double eval_uxx(const state_type& state);

	// Evaluate uxy pseudopotential term
	double eval_uxy(const state_type& state);

	// Evaluate uxz pseudopotential term
	double eval_uxz(const state_type& state);

	// Evaluate the uyy pseudopotential term
	double eval_uyy(const state_type& state);

	// Evaluate the uzz pseudopotential term
	double eval_uzz(const state_type& state);

	// Evaluate uyz pseudopotential term
	double eval_uyz(const state_type& state);

	// Evaluate the uyx Term
	double eval_uyx(const state_type& state);

	// Evaluate the uzx Term
	double eval_uzx(const state_type& state);

	// Evaluate the uzy Term
	double eval_uzy(const state_type& state);

	// Evaluate the jacobian A Matrix at the current state
	state_type A_mat(const state_type& state);
};

/* IntegrationObserver: Class for pulling states during integration.  
*  Objects of this class can be used as an observer for saving intermediate steps when integrating with boost.
*  As implemented, it provides no event function checking, it just saves integration results
*/
class IntegrationObserver
{
public:
	state_type t; // Vector for holding times at each observer call during integration
	state_type x; // Vector for holding states at each observer call during integration

	void operator()(const state_type& x_current, const double t_current)
	{
		t.push_back(t_current); // Save Time Values
		for (double i : x_current) // Save States
		{
			x.push_back(i);
		}
	}
};

/************************************ Function Definitions***************************************************/
// Define MexFunction::operator()
void MexFunction::operator()(ArgumentList outputs, ArgumentList inputs)
{
	// Validate Arguments
	validate_args(outputs, inputs);

	// Extract Input Info: States
	TypedArray<double> x0_matlab = (TypedArray<double>) inputs[0];
	TypedArray<double> tspan_matlab = (TypedArray<double>) inputs[1];
	double mu = inputs[2][0];

	// Stepping Parameters (Default)
	double AbsTol = 1e-16;
	double RelTol = 1e-13;
	double InitStep = 1e-10;

	if (inputs.size() == 6) // Stepping parameters specified
	{
		AbsTol = inputs[3][0];
		RelTol = inputs[4][0];
		InitStep = inputs[5][0];
	}

	// Convert States & Times to C++ Types for Use with Boost using Iterators
	// We now have std::vectors containing copies of the data in x0_matlab and tspan_matlab
	state_type x0(x0_matlab.begin(), x0_matlab.end());
	state_type tspan(tspan_matlab.begin(), tspan_matlab.end());

	// Create CR3BP System and Observer
	CR3BP system(mu); // For equations of motion with mass parameter
	IntegrationObserver obs; // To pass to integrator (by ref) to store intermediate integration results

	// Setup Integration Stepper
	// We make a controlled_runge_kutt<rk78> object, using make_controlled for an object of the same type.
	// We then set the absolute tolerance to AbsTol, and relative tolerance to RelTol
	odeint::controlled_runge_kutta<rk78> stepper = odeint::make_controlled<rk78>(AbsTol, RelTol);

	// Integrate
	if (tspan_matlab.getNumberOfElements() == 2) // Only start and end times specified
	{
		odeint::integrate_adaptive(stepper, system, x0, tspan[0], tspan[1], InitStep, std::ref(obs));
	}
	else { // Evaluate at all specified times using integrate_times
		odeint::integrate_times(stepper, system, x0, tspan, InitStep, std::ref(obs));
	}

	// Convert States Back to MATLAB Types
	ArrayFactory af; // For Generating matlab::Array types

	TypedArray<double> time_history = af.createArray<state_type::iterator>(ArrayDimensions({ obs.t.size(), 1 }), 
		obs.t.begin(), obs.t.end()); // Time history
	Array state_history; // Initialize state history

	if (x0.size() == 6) // Dynamics only
	{
		state_history = af.createArray<state_type::iterator>(ArrayDimensions({ obs.t.size(), 6 }),
			obs.x.begin(), obs.x.end(), InputLayout::ROW_MAJOR);
	}
	else //Include variational equations
	{
		state_history = af.createArray<state_type::iterator>(ArrayDimensions({ obs.t.size(), 42 }),
			obs.x.begin(), obs.x.end(), InputLayout::ROW_MAJOR);
	}

	// Set Outputs
	outputs[0] = time_history;
	outputs[1] = state_history;

}

// Define MexFunction::validate_args
void MexFunction::validate_args(ArgumentList outputs, ArgumentList inputs)
{
	ArrayFactory af; // For generating matlab::data::Array types
	std::shared_ptr<matlab::engine::MATLABEngine> engine_ptr = getEngine(); // For Throwing Errors if Necessary

	// Validate First Input Arguent (x0, the initial state)
	if (inputs[0].getType() != matlab::data::ArrayType::DOUBLE ||
		(inputs[0].getDimensions() != matlab::data::ArrayDimensions({ 6, 1 }) && 
			inputs[0].getDimensions() != ArrayDimensions({ 42, 1 })))
	{
		engine_ptr->feval(u"error", 0, std::vector<Array>({ af.createScalar(
			"Invalid initial state! Must be 6x1 double column vector!") }));
	}

	// Validate Second Input Argument (t_span, vector of time values)
	if (inputs[1].getType() != matlab::data::ArrayType::DOUBLE ||
		inputs[1].getNumberOfElements() < 2)
	{
		engine_ptr->feval(u"error", 0, std::vector<Array>({ af.createScalar(
			"Invalid time specification!  Must be a double array with at least 2 values!") }));
	}

	// Validate Third Input Argument (mu, mass parameter, scalar)
	if (inputs[2].getType() != matlab::data::ArrayType::DOUBLE ||
		inputs[2].getNumberOfElements() != 1)
	{
		engine_ptr->feval(u"error", 0, std::vector<Array>({ af.createScalar(
			"Invalid Mass Parameter!  Must be a scalar double") }));
	}

	// Validate Optional Arguments (if Present)
	if (inputs.size() > 3)
	{
		// Number of Elements
		if (inputs.size() != 6)
		{
			engine_ptr->feval(u"error", 0, std::vector<Array>({ af.createScalar(
			"Invalid Number of Optional Args!  Must specify AbsTol, RelTol, and Initial Step") }));
		}
		// Absolute Tolerance
		if (inputs[3].getType() != ArrayType::DOUBLE ||
			inputs[3].getNumberOfElements() != 1)
		{
			engine_ptr->feval(u"error", 0, std::vector<Array>({ af.createScalar(
			"Invalid Absolute Tolerance!  Must be a scalar double") }));
		}

		// Relative Tolerance
		if (inputs[4].getType() != ArrayType::DOUBLE ||
			inputs[4].getNumberOfElements() != 1)
		{
			engine_ptr->feval(u"error", 0, std::vector<Array>({ af.createScalar(
			"Invalid Relative Tolerance!  Must be a scalar double") }));
		}

		// Initial Step
		if (inputs[5].getType() != ArrayType::DOUBLE ||
			inputs[5].getNumberOfElements() != 1)
		{
			engine_ptr->feval(u"error", 0, std::vector<Array>({ af.createScalar(
			"Invalid Initial Step!  Must be a scalar double") }));
		}
	}

}

// Define CR3BP Constructor
CR3BP::CR3BP(double mu)
{
	m_mu = mu;
}

// Define CR3BP Operator
void CR3BP::operator()(const state_type& state, state_type& state_dot, const double t)
{
	if (state.size() == 6)
	{
		eom6(state, state_dot, t);
	}
	else if (state.size() == 42)
	{
		eom42(state, state_dot, t);
	}
}

// Define CR3BP Equations of Motion (State Only)
void CR3BP::eom6(const state_type& state, state_type& state_dot, const double t)
{
	// Unpack state for readability
	double x = state[0];
	double y = state[1];
	double z = state[2];
	double xd = state[3];
	double yd = state[4];
	double zd = state[5];

	// Define Distances from Primaries
	double d = sqrt(pow(x + m_mu, 2) + pow(y, 2) + pow(z, 2)); // Distance from larger primary
	double r = sqrt(pow(x - 1 + m_mu, 2) + pow(y, 2) + pow(z, 2)); // Distance from smaller primary

	// Equations of Motion
	state_dot[0] = xd;
	state_dot[1] = yd;
	state_dot[2] = zd;

	state_dot[3] = 2 * yd + x - ((1 - m_mu) * (x + m_mu)) / pow(d, 3) - (m_mu * (x - 1 + m_mu)) / pow(r, 3);
	state_dot[4] = -2 * xd + y - ((1 - m_mu) * y) / pow(d, 3) - (m_mu * y) / pow(r, 3);
	state_dot[5] = -((1 - m_mu) * z) / pow(d, 3) - (m_mu * z) / pow(r, 3);
}

// Define CR3BP Equations of Motion (w/ 1st Order Variations)
void CR3BP::eom42(const state_type& state, state_type& state_dot, const double t)
{
	// Initialize State Derivatives
	eom6(state, state_dot, t);

	// Get A Matrix
	state_type A = A_mat(state); // Row major layout


	// Linear Variational Derivatives
	int aug_state_index; // Index for current state element in state_dot
	int state_size6 = 6; // Size of pos+vel state

	// Perform multiplication Phidot = APhi, using vectors & linear indices
	// Elements 6:end of state_dot are in COLUMN MAJOR order
	for (int j = 0; j < state_size6; j++)
	{
		for (int i = 0; i < state_size6; i++)
		{
			aug_state_index = state_size6 + j * state_size6 + i;
			state_dot[aug_state_index] = 0.0;

			for (int k = 0; k < state_size6; k++)
			{
				state_dot[aug_state_index] += A[i * state_size6 + k] * state[state_size6 + j * state_size6 + k];
			}
		}
	}
}

/******************************** CR3BP Additional Member Function Definitions ***************************************/

// Compute distance from second primary
double CR3BP::eval_r(const state_type& state)
{
	return sqrt(pow(state[0] - 1 + m_mu, 2) + pow(state[1], 2) + pow(state[2], 2)); // Distance from smaller primary
}

// Compute distance from first primary
double CR3BP::eval_d(const state_type& state)
{
	return sqrt(pow(state[0] + m_mu, 2) + pow(state[1], 2) + pow(state[2], 2));
}

// Compute Pseudopotential Terms:
double CR3BP::eval_uxx(const state_type& state)
{
	double r = eval_r(state);
	double d = eval_d(state);
	double x = state[0];
	double y = state[1];
	double z = state[2];

	double uxx = 1 - (1 - m_mu) / pow(d, 3) - m_mu / pow(r, 3);
	uxx += (3 * (1 - m_mu) * pow(x + m_mu, 2)) / pow(d, 5);
	uxx += (3 * m_mu * pow(x - 1 + m_mu, 2)) / pow(r, 5);
	return uxx;
}

double CR3BP::eval_uxy(const state_type& state)
{
	double r = eval_r(state);
	double d = eval_d(state);
	double x = state[0];
	double y = state[1];
	double z = state[2];

	double uxy = (3 * (1 - m_mu) * (x + m_mu) * y) / pow(d, 5);
	uxy += (3 * m_mu * (x - 1 + m_mu) * y) / pow(r, 5);

	return uxy;
}

double CR3BP::eval_uxz(const state_type& state)
{
	double r = eval_r(state);
	double d = eval_d(state);
	double x = state[0];
	double y = state[1];
	double z = state[2];

	double uxz = (3 * (1 - m_mu) * (x + m_mu) * z) / pow(d, 5);
	uxz += (3 * m_mu * (x - 1 + m_mu) * z) / pow(r, 5);
	return uxz;
}

double CR3BP::eval_uyy(const state_type& state)
{
	double r = eval_r(state);
	double d = eval_d(state);
	double x = state[0];
	double y = state[1];
	double z = state[2];

	double uyy = 1 - (1 - m_mu) / pow(d, 3) - m_mu / pow(r, 3);
	uyy += (3 * (1 - m_mu) * pow(y, 2)) / pow(d, 5);
	uyy += 3 * m_mu * pow(y, 2) / pow(r, 5);

	return uyy;
}

double CR3BP::eval_uyz(const state_type& state)
{
	double r = eval_r(state);
	double d = eval_d(state);
	double x = state[0];
	double y = state[1];
	double z = state[2];

	double uyz = (3 * (1 - m_mu) * y * z) / pow(d, 5);
	uyz += 3 * m_mu * y * z / pow(r, 5);

	return uyz;
}

double CR3BP::eval_uzz(const state_type& state)
{
	double r = eval_r(state);
	double d = eval_d(state);
	double x = state[0];
	double y = state[1];
	double z = state[2];

	double uzz = -(1 - m_mu) / pow(d, 3) - m_mu / pow(r, 3);
	uzz += (3 * (1 - m_mu) * pow(z, 2)) / pow(d, 5);
	uzz += 3 * m_mu * pow(z, 2) / pow(r, 5);

	return uzz;
}

double CR3BP::eval_uyx(const state_type& state)
{
	return eval_uxy(state);
}

double CR3BP::eval_uzx(const state_type& state)
{
	return eval_uxz(state);
}

double CR3BP::eval_uzy(const state_type& state)
{
	return eval_uyz(state);
}

state_type CR3BP::A_mat(const state_type& state)
{
	state_type A(36, 0);

	// Position Derivatives
	A[3] = 1.0;
	A[10] = 1.0;
	A[17] = 1.0;

	// Pseudopotential Terms
	A[18] = eval_uxx(state);
	A[19] = eval_uxy(state);
	A[20] = eval_uxz(state);

	A[24] = eval_uyx(state);
	A[25] = eval_uyy(state);
	A[26] = eval_uyz(state);

	A[30] = eval_uzx(state);
	A[31] = eval_uzy(state);
	A[32] = eval_uzz(state);

	// Velocity Terms
	A[22] = 2.0;
	A[27] = -2.0;

	return A;
}