/*
 * Shape adjuster using keypoint likelihoods from a single image, after pose has been adjusted
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

	if(argc < 2){
		inputFileName = "ceres_input_singleViewShapeAdjuster.txt";
		//	inputFileName = "../data/ceres_input_singleViewShapeAdjuster.txt";
		outputFileName = "ceres_output_singleViewShapeAdjuster.txt";
	}
	else if(argc < 3){
		outputFileName = "ceres_output_singleViewShapeAdjuster.txt";
	}
	else{
		inputFileName = argv[1];
		outputFileName = argv[2];
	}

	// Create a 'SingleViewPoseAdjustmentProblem' instance to hold the BA problem
	SingleViewShapeAdjustmentProblem myProblem;
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
	double xzNormal[3] = {0, -1, 0};

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


	// Get the rotation and translation estimates (after PnP)
	double *rot = myProblem.getRot();
	double *trans = myProblem.getTrans();
	// Convert the rotation estimate to an axis-angle representation
	double rotAngleAxis[3] = {0.0, 0.001, 0.0};
	ceres::RotationMatrixToAngleAxis(rot, rotAngleAxis);
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

		// Create a cost function for the lambda reprojection error term
		// i.e., std reprojection error, but instead of 3D points and R,t, we solve for lambdas (shape params)


		//ceres::CostFunction *lambdaError = new ceres::AutoDiffCostFunction<LambdaReprojectionError, 2, 3, 5>(
			//new LambdaReprojectionError(X_bar+3*i, observations+2*i, curEigVec, K, observationWeights[i], trans));


		ceres::CostFunction *lambdaError = new ceres::AutoDiffCostFunction<LambdaReprojectionError, 2, 3, 5>(
			new LambdaReprojectionError(X_bar+3*i, observations+2*i, curEigVec, K, observationWeights[i], trans));




		// Add a residual block to the problem
		problem.AddResidualBlock(lambdaError, new ceres::HuberLoss(0.5), rotAngleAxis, lambdas);

		// Add a regularizer (to prevent lambdas from growing too large)
		ceres::CostFunction *lambdaRegularizer = new ceres::AutoDiffCostFunction<LambdaRegularizer, 3, 5>(
			new LambdaRegularizer(curEigVec));
		// Add a residual block to the problem
		problem.AddResidualBlock(lambdaRegularizer, new ceres::HuberLoss(0.10), lambdas);

		// // Create a cost function to regularize 3D keypoint locations (alignment error)
		// ceres::CostFunction *alignmentError = new ceres::AutoDiffCostFunction<LambdaAlignmentError, 3, 5>(
		// 	new LambdaAlignmentError(X_bar+3*i, observations+2*i, curEigVec, K, observationWeights[i], X_bar_initial+3*i));

	}
	// We don't want to optimize over the rotation here. We want it to remain the same as it was after PnP.
	problem.SetParameterBlockConstant(rotAngleAxis);

	// // Add a regularizer (to prevent lambdas from growing too large)
	// // This regularizer has different weights for each lambda
	// // double lambdaWeights[5] = {2, 1.5, 1.5, 1 ,1};
	// double lambdaWeights[5] = {2, 1.5, 1.5, 1 ,1};
	// ceres::CostFunction *lambdaRegularizerWeighted = new ceres::AutoDiffCostFunction<LambdaRegularizerWeighted, 5, 5>(
	// 	new LambdaRegularizerWeighted(lambdaWeights));
	// // Add a residual block to the problem
	// problem.AddResidualBlock(lambdaRegularizerWeighted, new ceres::HuberLoss(1), lambdas);

	// // Set (better) lower-bounds on lambdas
	// problem.SetParameterLowerBound(lambdas, 0, -0.75);
	// problem.SetParameterUpperBound(lambdas, 0, 0.75);
	// problem.SetParameterLowerBound(lambdas, 1, -0.8);
	// problem.SetParameterUpperBound(lambdas, 1, 0.8);
	// problem.SetParameterLowerBound(lambdas, 2, -0.8);
	// problem.SetParameterUpperBound(lambdas, 2, 0.8);
	// problem.SetParameterLowerBound(lambdas, 3, -0.8);
	// problem.SetParameterUpperBound(lambdas, 3, 0.8);
	// problem.SetParameterLowerBound(lambdas, 4, -0.8);
	// problem.SetParameterUpperBound(lambdas, 4, 0.8);


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
	// options.use_nonmonotonic_steps = true;
	// options.function_tolerance = 1e-5;
	// // Optionally, specify callbacks to be executed each iter
	// VarCallback varCallback;
	// options.callbacks.push_back(&varCallback);
	// options.update_state_every_iteration = true;
  const char *FileShapeOutput = "lambdas.txt";
  std::ofstream sfile;
  sfile.open(FileShapeOutput, std::ios_base::app);

	// Solve the problem and print the results
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
  // std::cout << "Shape adjustment initial cost " << summary.initial_cost << std::endl;
  sfile << lambdas[0] << " " << lambdas[1] << " " << lambdas[2] << " " << lambdas[3] << " " << lambdas[4] << std::endl;
  sfile.close();
//   std::cout << summary.FullReport() << "\n"; 


	// Open the output file (to write the result)
	std::ofstream outFile;
	outFile.open(outputFileName);

	// // Print the values of lambdas
	// std::cout << lambdas[0] << " " << lambdas[1] << " " << lambdas[2] << " " << lambdas[3] << " " << lambdas[4] << std::endl;
	// std::cout << "rotAngleAxis: " << rotAngleAxis[0] << " " << rotAngleAxis[1] << " " << rotAngleAxis[2] << std::endl;

	// Compute the resultant 3D wireframe
	for(int i = 0; i < numPts; ++i){
		double temp[3];
		temp[0] = X_bar_initial[3*i];
		temp[1] = X_bar_initial[3*i+1];
		temp[2] = X_bar_initial[3*i+2];

		for(int j = 0; j < 5; ++j){
			temp[0] += lambdas[j]*V[3*numObs*j + 3*i + 0];
			temp[1] += lambdas[j]*V[3*numObs*j + 3*i + 1];
			temp[2] += lambdas[j]*V[3*numObs*j + 3*i + 2];
			// std::cout << V[3*numObs*j + 3*i + 0] << " " << V[3*numObs*j + 3*i + 1] << " " << \
			// 	V[3*numObs*j + 3*i + 2] << std::endl;
		}

		ceres::AngleAxisRotatePoint(rotAngleAxis, temp, temp);
		temp[0] += trans[0];
		temp[1] += trans[1];
		temp[2] += trans[2];

		// Write the output to file
		outFile << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;

		// // Print the output to stdout
		// std::cout << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
	}
	std::cout << lambdas[0] << " " << lambdas[1] << " " << lambdas[2] << " " << lambdas[3] << " " << lambdas[4] << std::endl;
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
