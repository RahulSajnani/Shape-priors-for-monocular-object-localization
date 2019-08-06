/*
 * Pose adjustmer using keypoint likelihoods from a single image
 * Author: Krishna Murthy
*/

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <ceres/loss_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/rotation.h>

// Contains definitions of various problem structs
#include "problemStructs.hpp"
// Contains various cost function struct specifications
#include "costFunctions.hpp"


int main(int argc, char** argv){

	google::InitGoogleLogging(argv[0]);

	const char *inputFileName;
	const char *outputFileName;
  const char *FilePoseOutput = "pose_cost.txt";


	if(argc < 2){
		inputFileName = "ceres_input_singleViewPoseAdjuster.txt";
		outputFileName = "ceres_output_singleViewPoseAdjuster.txt";
	}
	else if(argc < 3){
		outputFileName = "ceres_output_singleViewPoseAdjuster.txt";
	}
	else{
		inputFileName = argv[1];
		outputFileName = argv[2];
	}

	// Create a 'SingleViewPoseAdjustmentProblem' instance to hold the BA problem
	SingleViewPoseAdjustmentProblem myProblem;
	// Read the problem file and store relevant information
	if(!myProblem.loadFile(inputFileName)){
		std::cerr << "ERROR: Unable to open file " << inputFileName << std::endl;
		return 1;
	}

	// Get a pointer to the observation vector/matrix from the BA problem
	const int numObs = myProblem.getNumObs();
	const int numPts = myProblem.getNumPts();
	double *observations = myProblem.observations();
	double *observationWeights = myProblem.observationWeights();
	double *K = myProblem.getK();
	double *X_bar = myProblem.getX_bar();
	// Store the initial X_bar(s)
	double *X_bar_initial;
	X_bar_initial = new double[3*numPts];
	for(int i = 0; i < 3*numObs; ++i){
		X_bar_initial[i] = X_bar[i];
	}
	// Variable to store the initial and final reprojection errors
	double initialReprojError = 0.0, finalReprojError = 0.0;

	// Normal to the XZ plane (ground plane)
	double xzNormal[3] = {0, 1, 0};

	// Get the center of the car
	double *carCenter = myProblem.getCarCenter();
	// Get the length, width, and height of the car
	const double carLength = myProblem.getCarLength();
	const double carWidth = myProblem.getCarWidth();
	const double carHeight = myProblem.getCarHeight();

	// Get the top 5 eigenvectors of the wireframe
	double *V = myProblem.getV();
	// Get the weights of the linear combination
	double *lambdas = myProblem.getLambdas();


	// Define start indices of various keypoints
	const int L_F_WHEELCENTER = 0;
	const int R_F_WHEELCENTER = 3;
	const int L_B_WHEELCENTER = 6;
	const int R_B_WHEELCENTER = 9;
	const int L_HEADLIGHT = 12;
	const int R_HEADLIGHT = 15;
	const int L_TAILLIGHT = 18;
	const int R_TAILLIGHT = 21;
	const int L_SIDEVIEWMIRROR = 24;
	const int R_SIDEVIEWMIRROR = 27;
	const int L_F_ROOFTOP = 30;
	const int R_F_ROOFTOP = 33;
	const int L_B_ROOFTOP = 36;
	const int R_B_ROOFTOP = 39;


	// Initialize the rotation and translation estimates
	double rotMat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	double trans[3] = {0.1, 0.1, 0.1};
	trans[0] = carCenter[0];
	trans[1] = carCenter[1];
	trans[3] = carCenter[2];
	// Convert the rotation estimate to an axis-angle representation
	double rotAngleAxis[3] = {0.001, 1, 0.001};
	// ceres::RotationMatrixToAngleAxis(rot, rotAngleAxis);
	// std::cout << "rotAngleAxis: " << rotAngleAxis[0] << " " << rotAngleAxis[1] << " " << rotAngleAxis[2] << std::endl;



	// -----------------------------------
	// Construct the Optimization Problem
	// -----------------------------------


	// Declare a Ceres problem instance to hold cost functions
	ceres::Problem problem;

	// For each observation, add a standard PnP error (reprojection error) residual block
	for(int i = 0; i < numObs; ++i){
		// Create a vector of eigenvalues for the current keypoint
		double *curEigVec = new double[15];
		// std::cout << "curEigVec: ";
		for(int j = 0; j < 5; ++j){
			curEigVec[3*j+0] = V[3*numObs*j + 3*i + 0];
			curEigVec[3*j+1] = V[3*numObs*j + 3*i + 1];
			curEigVec[3*j+2] = V[3*numObs*j + 3*i + 2];
			// std::cout << V[3*numObs*j + 3*i + 0] << " " << V[3*numObs*j + 3*i + 1] << " " << \
			// 	V[3*numObs*j + 3*i + 2] << std::endl;
		}
		// Create a cost function for the PnP error term
		ceres::CostFunction *pnpError = new ceres::AutoDiffCostFunction<PnPError, 2, 3, 3>(
			new PnPError(X_bar_initial+3*i, observations+2*i, curEigVec, K, observationWeights[i], lambdas));
		// Add a residual block to the problem
		// ceres::HuberLoss(0.8) worked for most cases
		problem.AddResidualBlock(pnpError, new ceres::HuberLoss(2.0), rotAngleAxis, trans);
	}


	// Add a regularizer to the translation term (to prevent a huge drift from the initialization)
	ceres::CostFunction *translationRegularizer = new ceres::AutoDiffCostFunction<TranslationRegularizer, 3, 3>(
		new TranslationRegularizer(carCenter));
	problem.AddResidualBlock(translationRegularizer, new ceres::HuberLoss(0.2), trans);

	// Add a rotation regularizer, to ensure that the rotation is about the Y-axis
	ceres::CostFunction *rotationRegularizer = new ceres::AutoDiffCostFunction<RotationRegularizer, 3, 3>(
		new RotationRegularizer());
	problem.AddResidualBlock(rotationRegularizer, NULL, rotAngleAxis);


	// Set lower and upper bounds on the translation estimates returned
	problem.SetParameterLowerBound(trans, 0, -4);
	problem.SetParameterUpperBound(trans, 0, 4);
	problem.SetParameterLowerBound(trans, 1, -4);
	problem.SetParameterUpperBound(trans, 1, 4);
	problem.SetParameterLowerBound(trans, 2, -7);
	problem.SetParameterUpperBound(trans, 2, 7);


	// -----------------------------------
	// Solve the Optimization Problem
	// -----------------------------------


	// Specify solver options
	ceres::Solver::Options options;
	// options.linear_solver_type = ceres::DENSE_SCHUR;
	// ITERATIVE_SCHUR > DENSE_SCHUR ~= SPARSE_SCHUR
	// options.linear_solver_type = ceres::ITERATIVE_SCHUR;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.preconditioner_type = ceres::JACOBI;
	// ITERATIVE_SCHUR + explicit schur complement = disaster
	// options.use_explicit_schur_complement = true;
	// options.use_inner_iterations = true;
  	options.max_num_iterations = 100;
	options.minimizer_progress_to_stdout = false;
	// options.function_tolerance = 1e-5;

	// // Optionally, specify callbacks to be executed each iter
	// VarCallback varCallback;
	// options.callbacks.push_back(&varCallback);
	// options.update_state_every_iteration = true;
  std::ofstream pfile;
  pfile.open(FilePoseOutput);
	// Solve the problem and print the results
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
  pfile << summary.initial_cost/4 <<  " " << summary.final_cost/4 << std::endl;
	// std::cout << "Pose adjustment cost" << summary.final_cost/4 << std::endl;
  pfile.close();


	// Open the output file (to write the result)
	std::ofstream outFile;
	outFile.open(outputFileName);


	// Write the estimated rotation and translation to the output file
	ceres::AngleAxisToRotationMatrix(rotAngleAxis, rotMat);
	for(int i = 0; i < 9; ++i){
		// Write out an entry of the estimated rotation matrix to file (in column-major order)
		outFile << rotMat[i] << std::endl;
		// // Display the same on stdout
		// std::cout << rotMat[i] << std::endl;
	}
	// Write the translation estimate to file
	outFile << trans[0] << std::endl;
	outFile << trans[1] << std::endl;
	outFile << trans[2] << std::endl;
	// // Print the translation estimate to stdout
	// std::cout << trans[0] << std::endl;
	// std::cout << trans[1] << std::endl;
	// std::cout << trans[2] << std::endl;
	// // Print the rotation (axis-angle) estimate to stdout
	// std::cout << rotAngleAxis[0] << " " << rotAngleAxis[1] << " " << rotAngleAxis[2] << std::endl;

	// // Compute the resultant 3D wireframe
	// for(int i = 0; i < numPts; ++i){
	// 	double temp[3];
	// 	temp[0] = X_bar_initial[3*i];
	// 	temp[1] = X_bar_initial[3*i+1];
	// 	temp[2] = X_bar_initial[3*i+2];
	// 	// Rotate 'temp' using the optimized rotation, and store the result back in 'temp'
	// 	ceres::AngleAxisRotatePoint(rotAngleAxis, temp, temp);
	// 	// Add the translation
	// 	temp[0] += trans[0];
	// 	temp[1] += trans[1];
	// 	temp[2] += trans[2];
	// 	// Write the output to file
	// 	outFile << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
	// }
	// std::cout << "rot: " << rotAngleAxis[0] << " " << rotAngleAxis[1] << " " << rotAngleAxis[2] << std::endl;
	// std::cout << "trans: " << trans[0] << " " << trans[1] << " " << trans[2] << std::endl;


	// // Write output to file
	// std::ofstream outFile;
	// outFile.open(outputFileName);
	// for(int i = 0; i < numPts; ++i){
	// 	outFile << Q[3*i+0] << " " << Q[3*i+1] << " " << Q[3*i+2] << std::endl;
	// }


 	return 0;

}
