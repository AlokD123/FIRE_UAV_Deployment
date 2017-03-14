#include <cstdio>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <Python.h>
#include <stdlib.h>

void getUltraNGasData(int* ultra_dist, int* gas_density,long first_time){
	// Create some Python objects that will later be assigned values.
	PyObject *pName, *pModule, *pDict, *pFunc, *pArgs;
	// Set Python path
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('/home/pi/Desktop')");

	// Convert the file name to a Python string.
	pName = PyString_FromString("GetData_Ultra+Gas_BAREBONES");
	if(pName == NULL){
		printf("Mispelled script name?\n");
		exit(-1);
	}

	// Import the file as a Python module.
	pModule = PyImport_Import(pName);
	if(pModule == NULL){
		printf("No Python script found!\n");
		exit(-1);
	}

	// Create a dictionary for the contents of the module.
	pDict = PyModule_GetDict(pModule);
	// Get the add method from the dictionary.
	pFunc = PyDict_GetItemString(pDict, "main");

	if(pFunc == NULL){
		printf("No function.\n");
		exit(-1);
	}

	//pArgs=PyBool_FromLong(first_time);
	
	Py_INCREF(pName);
	Py_INCREF(pModule);
	Py_INCREF(pDict);
	Py_INCREF(pFunc);
	//Py_INCREF(pArgs);

	printf("Before call script\n");
	// Call the function with the arguments.
	PyObject* pResult = PyObject_CallObject(pFunc,NULL);//pArgs);
	printf("After call script\n");
	// Print a message if calling the method failed.
	if(pResult == NULL){
		printf("Method failed.\n");
		exit(-2);
	}

	//Parse values
	PyObject* tupleItem1 = PyTuple_GetItem(pResult,0);
	if(tupleItem1 == NULL){
			printf("No ultrasonic distance return value from Python script.\n");
		exit(-2);
	}else{
		*ultra_dist=(int) PyInt_AsLong(tupleItem1);
	}
	PyObject* tupleItem2 = PyTuple_GetItem(pResult,1);
	if(tupleItem2 == NULL){
			printf("No gas sensor measurement return value from Python script.\n");
		exit(-2);
	}else{
		*gas_density=(int) PyInt_AsLong(tupleItem2);
	}
	
	Py_CLEAR(pModule);
	Py_CLEAR(pName);
	Py_CLEAR(pDict);
	Py_CLEAR(pFunc);
	//Py_CLEAR(pArgs);
	Py_CLEAR(pResult);
	Py_CLEAR(tupleItem1);
	Py_CLEAR(tupleItem2);

	printf("Ultra_dist = %d, Gas_density = %d\n",*ultra_dist,*gas_density);
}



int main(){
int ultra_dist=0;		//Ultrasonic measured range (in cm)
int gas_density=0;		//Estimate of gas density (%)

	// Initialize the Python interpreter.
	Py_Initialize();

	long first_time=1;
	while(1){
		getUltraNGasData(&ultra_dist,&gas_density,first_time);	//Get measurements from gas and ultrasonic sensors by calling Python script
		first_time=0;
	}
	// Destroy the Python interpreter upon exit (ideally never).
	Py_Finalize();
}