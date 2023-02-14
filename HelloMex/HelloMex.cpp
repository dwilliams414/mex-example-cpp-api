/* Hello Mex
* This is a simple Mex function to ensure that you are configured to mex correctly and illustrate several key aspects of
* a mex function.  It also shows how iterators can be used to convert MATLAB defined types from the C++ Mex API into more 
* standard C++ types (like strings).
* 
* Function Signature (in MATLAB)
*		[char_out1, char_out2] = HelloMex(name)
* Here, char_out1, char_out2, and name are character arrays (e.g. 'CR3BP')
*/

// Include Headers necessary for MEX interface
#include "mex.hpp" 
#include "mexAdapter.hpp"

using matlab::mex::ArgumentList; // For EZ use of the Argument List Object

/* Create a class that inherits from matlab::mex::Function.  Your source file MUST define this class as follows
* and with the name MexFunction.  The name of the function call in MATLAB is determined by the name of source file.
*/
class MexFunction : public matlab::mex::Function
{
public:
	/* To write a mex function, we define the MexFunction's operator() function, which must take
	*  two arguments of type matlab::mex::ArgumentList, which correspond to the inputs to the function from MATLAB and the outputs
	*/
	void operator()(ArgumentList outputs, ArgumentList inputs)
	{
		matlab::data::ArrayFactory af; // Array Factory object.  Useful for turning C++ data types into TypedArrays that can
									   // be returned to MATLAB

		// Check Input Arguments (See Below)
		ArgumentValidation(outputs, inputs);

		// Generate MATLAB Character array from first input argument
		matlab::data::CharArray my_char_array= inputs[0];

		/*Create a C++ std::string to operate with natively in C++
		We can do this using iterators, which can also be used for other array types
		Using this iterator (or range) constructor creates a new object initialized
		with values defined by the begin and end iterators of the old/other object.  
		Changing the values in one object does not affect the other (since we are dealing
		with value types)
		*/
		std::string my_str(my_char_array.begin(), my_char_array.end());

		// Copy our string, to illustrate that my_str, my_changed_str, and my_char_array
		// do not reference eachother in memory
		std::string my_changed_str(my_str.begin(), my_str.end()); 
		my_changed_str[0] = '\n'; // Change my_changed_str value, to show that my_str is not affected

		// Return Results: We must set the elements of outputs to (MATLAB) Array Types.  To do this
		// we can use an array factory
		outputs[0] = af.createCharArray("Hello " + my_str); // Output for my_str
		outputs[1] = af.createCharArray("Hello " + my_changed_str); // Output for my_changed_str

	}

	// Argument Validation Example
	void ArgumentValidation(ArgumentList outputs, ArgumentList inputs)
	{
		// Get a pointer to the matlab running engine, to allow for throwing errors due to 
		// invalid input arguments
		std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
		matlab::data::ArrayFactory af;
		
		
		// Ensure that input is a character array
		if (inputs[0].getType() != matlab::data::ArrayType::CHAR)
		{
			matlabPtr->feval(u"error", 0, std::vector<matlab::data::Array>{ af.createScalar("Must input character array!") });
		}
	}
};