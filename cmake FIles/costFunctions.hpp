#include <ceres/ceres.h>
#include <ceres/rotation.h>


// Define a struct to hold the distance error. This error specifies that the solution mesh
// must be close to the control (initial) mesh.
struct DistanceError{

	// Constructor
	DistanceError(double *P) : P_(P) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const Q, T* residuals) const {
		residuals[0] = T(2)*(Q[0] - T(P_[0]));
		residuals[1] = T(2)*(Q[1] - T(P_[1]));
		residuals[2] = T(2)*(Q[2] - T(P_[2]));
		return true;
	}

	// Control Mesh
	double *P_;

};


// Define a struct to hold the planarity error.
struct PlanarityError{

	// Constructor
	PlanarityError() {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	// The inputs are the plane normal n, distance from the origin d, and the vertices q
	template <typename T>
	bool operator() (const T* const n, const T* const d, const T* const q0, \
		const T* const q1, const T* const q2, const T* const q3, T* residuals) const {

		// Dot product must be zero, for planarity
		// residuals[0] = T(100) * ( (n[0]*q0[0] + n[1]*q0[1] + n[2]*q0[2] + d[0]) + \
		// 	(n[0]*q1[0] + n[1]*q1[1] + n[2]*q1[2] + d[0]) + (n[0]*q2[0] + n[1]*q2[1] + n[2]*q2[2] + d[0]) + \
		// 	 (n[0]*q3[0] + n[1]*q3[1] + n[2]*q3[2] + d[0]) );

		residuals[0] = T(10)*(n[0]*q0[0] + n[1]*q0[1] + n[2]*q0[2] + d[0]);
		residuals[1] = T(10)*(n[0]*q1[0] + n[1]*q1[1] + n[2]*q1[2] + d[0]);
		residuals[2] = T(10)*(n[0]*q2[0] + n[1]*q2[1] + n[2]*q2[2] + d[0]);
		residuals[3] = T(10)*(n[0]*q3[0] + n[1]*q3[1] + n[2]*q3[2] + d[0]);

		return true;
	}

};


// Struct to enforce the unit magnitude constraint (for plane normal vectors)
struct UnitMagnitude{

	// Constructor
	UnitMagnitude() {}

	// The operator method. Evaluates the residuals and computes the jacobians.
	template <typename T>
	bool operator() (const T* const n, T* residuals) const{
		residuals[0] = T(5) * ((n[0]*n[0] + n[1]*n[1] + n[2]*n[2]) - T(1.0));
		return true;
	}

};


// Define the keypoint reprojection error. We assume that the predicted shape
// is in camera coordinates, i.e., we are solving only for the four parameters
// of the linear combination of the deformation basis for each keypoint.
struct KeypointReprojectionError{

	KeypointReprojectionError(double *observations, double *K)
		: observations(observations), K(K) {}

	// The most important method of the struct.
	// The operator() method is called by the solver whenever the residual block is evaluated,
	// or whenever jacobians have to be computed.
	template <typename T>
	bool operator()(const T* const X_bar, T* residuals) const{

		T p[3];
		p[0] = T(K[0])*X_bar[0] + T(K[1])*X_bar[1] + T(K[2])*X_bar[2];
		p[1] = T(K[3])*X_bar[0] + T(K[4])*X_bar[1] + T(K[5])*X_bar[2];
		p[2] = T(K[6])*X_bar[0] + T(K[7])*X_bar[1] + T(K[8])*X_bar[2];

		T px = p[0] / p[2];
		T py = p[1] / p[2];

		T x_p = (T(K[0])*X_bar[0] + T(K[1])*X_bar[1] + T(K[2])*X_bar[2]) / (T(K[6])*X_bar[0] + T(K[7])*X_bar[1] + T(K[8])*X_bar[2]);
		T y_p = (T(K[3])*X_bar[0] + T(K[4])*X_bar[1] + T(K[5])*X_bar[2]) / (T(K[6])*X_bar[0] + T(K[7])*X_bar[1] + T(K[8])*X_bar[2]);

		// // Compute the residuals (this one works well)
		// residuals[0] = px - T(observations[0]);
		// residuals[1] = px - T(observations[1]);

		// Compute the residuals (works very well)
		residuals[0] = T(1)*(x_p - T(observations[0]));
		residuals[1] = T(1)*(y_p - T(observations[1]));

		// // Compute the residuals (this one doesn't seem to work at all)
		// residuals[0] = ((K[0]*X_bar[0] + K[1]*X_bar[1] + K[2]*X_bar[2]) / (K[6]*X_bar[0] + K[7]*X_bar[1] + K[8]*X_bar[2])) - T(observations[0]);
		// residuals[1] = ((K[3]*X_bar[0] + K[4]*X_bar[1] + K[5]*X_bar[2]) / (K[6]*X_bar[0] + K[7]*X_bar[1] + K[8]*X_bar[2])) - T(observations[1]);

		return true;
	}

	// // Factory to hide the construction of the CostFunction object from the client code
	// static ceres::CostFunction* Create(ouble *observations, double *K){
	// 	double *curObs;
	// 	curObs = new double[2];
	// 	curObs[0] = observations[0];
	// 	curObs[1] = observations[1];
	// 	return(new ceres::AutoDiffCostFunction<KeypointReprojectionError, 2, 3>(
	// 		new KeypointReprojectionError(curObs, K)));
	// }

	double *observations;
	double *K;
};


// Define the Chamfer field keypoint reprojection error. We assume that the predicted shape
// is in camera coordinates, i.e., we are solving only for the four parameters
// of the linear combination of the deformation basis for each keypoint.
// This is different from the regular keypoint reprojection error in the sense that, here,
// we treat the error as zero, if the projected point is within a particular distance of
// influence from the actual keypoint.
struct ChamferFieldKeypointError{

	ChamferFieldKeypointError(double *observations, double *K, double influence)
		: observations(observations), K(K), influence(influence) {}

	// The most important method of the struct.
	// The operator() method is called by the solver whenever the residual block is evaluated,
	// or whenever jacobians have to be computed.
	template <typename T>
	bool operator()(const T* const X_bar, T* residuals) const{

		T p[3];
		p[0] = T(K[0])*X_bar[0] + T(K[1])*X_bar[1] + T(K[2])*X_bar[2];
		p[1] = T(K[3])*X_bar[0] + T(K[4])*X_bar[1] + T(K[5])*X_bar[2];
		p[2] = T(K[6])*X_bar[0] + T(K[7])*X_bar[1] + T(K[8])*X_bar[2];

		T px = p[0] / p[2];
		T py = p[1] / p[2];

		T x_p = (T(K[0])*X_bar[0] + T(K[1])*X_bar[1] + T(K[2])*X_bar[2]) / (T(K[6])*X_bar[0] + T(K[7])*X_bar[1] + T(K[8])*X_bar[2]);
		T y_p = (T(K[3])*X_bar[0] + T(K[4])*X_bar[1] + T(K[5])*X_bar[2]) / (T(K[6])*X_bar[0] + T(K[7])*X_bar[1] + T(K[8])*X_bar[2]);

		// Compute the residuals (works very well)
		if(abs(x_p - T(observations[0])) <= T(influence)){
			residuals[0] = T(0.000000001);
		}
		else{
			residuals[0] = x_p - T(observations[0]);
		}
		if(abs(y_p - T(observations[1])) <= T(influence)){
			residuals[1] = T(0.000000001);
		}
		else{
			residuals[1] = y_p - T(observations[1]);
		}

		return true;
	}

	// // Factory to hide the construction of the CostFunction object from the client code
	// static ceres::CostFunction* Create(ouble *observations, double *K){
	// 	double *curObs;
	// 	curObs = new double[2];
	// 	curObs[0] = observations[0];
	// 	curObs[1] = observations[1];
	// 	return(new ceres::AutoDiffCostFunction<ChamferFieldKeypointError, 2, 3>(
	// 		new ChamferFieldKeypointError(curObs, K)));
	// }

	double *observations;
	double *K;
	// Distance of influence of the Chamfer field mask
	double influence;
};


// Define the weighted keypoint reprojection error. We assume that the predicted shape
// is in camera coordinates, i.e., we are solving only for the four parameters
// of the linear combination of the deformation basis for each keypoint.
// This weighting is useful while implementing the IRLS scheme.
struct WeightedKeypointReprojectionError{

	WeightedKeypointReprojectionError(double *observations, double *K, double weight)
		: observations_(observations), K_(K), weight_(weight) {}

	// The most important method of the struct.
	// The operator() method is called by the solver whenever the residual block is evaluated,
	// or whenever jacobians have to be computed.
	template <typename T>
	bool operator()(const T* const X_bar, T* residuals) const{

		T p[3];
		p[0] = T(K_[0])*X_bar[0] + T(K_[1])*X_bar[1] + T(K_[2])*X_bar[2];
		p[1] = T(K_[3])*X_bar[0] + T(K_[4])*X_bar[1] + T(K_[5])*X_bar[2];
		p[2] = T(K_[6])*X_bar[0] + T(K_[7])*X_bar[1] + T(K_[8])*X_bar[2];

		T px = p[0] / p[2];
		T py = p[1] / p[2];

		T x_p = (T(K_[0])*X_bar[0] + T(K_[1])*X_bar[1] + T(K_[2])*X_bar[2]) / (T(K_[6])*X_bar[0] + T(K_[7])*X_bar[1] + T(K_[8])*X_bar[2]);
		T y_p = (T(K_[3])*X_bar[0] + T(K_[4])*X_bar[1] + T(K_[5])*X_bar[2]) / (T(K_[6])*X_bar[0] + T(K_[7])*X_bar[1] + T(K_[8])*X_bar[2]);

		// // Compute the residuals (this one works well)
		// residuals[0] = px - T(observations[0]);
		// residuals[1] = px - T(observations[1]);

		// Compute the residuals (works very well)
		residuals[0] = T(sqrt(weight_))*(x_p - T(observations_[0]));
		residuals[1] = T(sqrt(weight_))*(y_p - T(observations_[1]));

		return true;
	}

	double *observations_;
	double *K_;
	double weight_;
};


// Define the regularization term in the cost function. This term encourages the 3D points
// to be close to the center of the car
struct RegularizationTerm{

	// Constructor
	RegularizationTerm(double *carCenter) : carCenter_(carCenter) {}

	// Operator method. Evaluates the cost function and computes Jacobians.
	template <typename T>
	bool operator()(const T* const X_bar, T* residuals) const{

		// Regularize with respect to car center
		// residuals[0] = abs(X_bar[0] - T(-6.0408));
		// residuals[1] = abs(X_bar[1] - T(2.2448));
		// residuals[2] = abs(X_bar[2] - T(22.5947));

		residuals[0] = T(10)*(X_bar[0] - T(carCenter_[0]));
		residuals[1] = T(10)*(X_bar[1] - T(carCenter_[1]));
		residuals[2] = T(10)*(X_bar[2] - T(carCenter_[2]));

		// Regularize with respect to initial location
		// residuals[0] = X_bar[0] - T(X_bar_initial_[0]);
		return true;
	}

	// Car center
	double *carCenter_;

};


// Define a cost function that encourages keypoints to retain their initial depths
struct InitialDepth{

	// Constructor
	InitialDepth(double Z_init) : Z_init_(Z_init) {}

	// Operator method. Evaluates the cost function and computes the Jacobians
	template <typename T>
	bool operator() (const T* const Z, T* residuals) const{
		residuals[0] = Z[0] - T(Z_init_);
		return true;
	}

	// Initial depth
	double Z_init_;

};


// Struct to enforce the constraint that a normal must be parallel to the ground plane normal.
// We assume that the XZ plane is the ground plane (from calibration).
struct ParallelToGround{

	// Constructor
	ParallelToGround() {}

	// The operator method. Evaluates the residuals and computes the jacobians.
	template <typename T>
	bool operator() (const T* const n, T* residuals) const {
		residuals[0] = T(3) * (n[0] - T(0.0));
		residuals[1] = T(3) * (n[1] - T(1.0));
		residuals[2] = T(3) * (n[2] - T(0.0));
		return true;
	}

};


// Define a 'box coverage' term. This term encourages the 3D points to be far apart from each other.
struct BoxCoverage{

	// Constructor
	BoxCoverage() {}

	// Operator method. Evaluates the cost function and computes Jacobians
	template <typename T>
	bool operator() (const T* const X1, const T* const X2, T* residuals) const{
		residuals[0] = T(1.0)/(X1[0]-X2[0]);
		residuals[1] = T(1.0)/(X1[1]-X2[1]);
		residuals[2] = T(1.0)/(X1[2]-X2[2]);
		return true;
	}

};


// Define a 'dimension prior'. This prior encourages the two 3D points to be of appropriate length.
struct DimensionPrior{

	// Constructor
	DimensionPrior(double mag) : mag_(mag) {}

	// Operator method. Evaluates the cost function and computes Jacobians.
	template <typename T>
	bool operator() (const T* const X1, const T* const X2, T* residuals) const{
		residuals[0] = T(10)*(pow(X1[0]-X2[0],2) + pow(X1[1]-X2[1],2) + pow(X1[2]-X2[2],2) - T(mag_)*T(mag_));
		return true;
	}

	// Magnitude that the points must have
	double mag_;
};


// Define a 'height prior'. This prior encourages the height to be of appropriate magnitude.
// This function is different from the dimension prior, because dimension prior is computed
// for two points, i.e., it penalizes the deviation of the distance between two points from its
// magnitude, whereas height prior penalizes the deviation of the expected heigth from the ground
// plane (assumed to be the XZ-plane).
struct HeightPrior{

	// Constructor
	HeightPrior(double mag) : mag_(mag) {}

	// Operator method. Evaluates the cost function and computes Jacobians.
	template <typename T>
	bool operator() (const T* const X, T* residuals) const{
		residuals[0] = T(5)*(X[1] - T(mag_));
		return true;
	}

	// Magnitude of height
	double mag_;

};


// Performs Laplacian smooting regularization. Takes as input the mean of the neighboring vertices
// of the mesh. Penalizes distances from that mean. Optionally allows for weighted penalization.
struct LaplacianSmoother{

	// Constructor
	LaplacianSmoother(double *meanPt) : mean_(meanPt) {}

	// Operator method. Evaluates the cost function and computes Jacobians.
	template <typename T>
	bool operator() (const T* const X, T* residuals) const{
		residuals[0] = X[0] - mean_[0];
		residuals[1] = X[1] - mean_[1];
		residuals[2] = X[2] - mean_[2];
		return true;
	}

	// Mean of neighboring points (i.e., that point from which (weighted) distance should be penalized)
	double *mean_;

};


// Cost function struct that takes in 4 points. Penalizes them if the line joining points 1,2 is not
// parallel to the line joining 3,4.
struct ParallelLines{

	// Constructor
	ParallelLines() {}

	// Operator method. Evaluates the cost function and computes Jacobians.
	template <typename T>
	bool operator() (const T* const X1, const T* const X2, const T* const X3, \
		const T* const X4, T* residuals) const{
		// Weighting factor
		T weight_ = T(1000);
		// Compute cross-product. It should be zero, for parallel (or-antiparallel) vectors
		residuals[0] = T(5)*((-(X2[2]-X1[2])*(X4[1]-X3[1]) + (X2[1]-X1[1])*(X4[2]-X3[2])) + \
			((X2[2]-X1[2])*(X4[0]-X3[0]) -(X2[0]-X1[0])*(X4[2]-X3[2])) + \
			(-(X2[1]-X1[1])*(X4[0]-X3[0]) + (X2[0]-X1[0])*(X4[1]-X3[1])));
		// if(residuals[0] > 0.2*((X2[0]-X1[0])*(X2[0]-X1[0]) + (X2[1]-X1[1])*(X2[1]-X1[1]) + \
		// 	(X2[2]-X1[2])*(X2[2]-X1[2])) * ((X4[0]-X3[0])*(X4[0]-X3[0]) + (X4[1]-X3[1])*(X4[1]-X3[1]) + \
		// 	(X4[2]-X3[2])*(X4[2]-X3[2])) ){
		// 		return false;
		// }
		return true;
	}

};


// Cost function struct that takes in 3 points. Penalizes them if the line joining points 1,2 is not
// perpendicular to the line joining 2,3.
struct PerpendicularLines{

	// Constructor
	PerpendicularLines() {}

	// Operator method. Evaluates the cost function and computes Jacobians
	template <typename T>
	bool operator() (const T* const X1, const T* const X2, const T* const X3, T* residuals) const{
		// Weighting factor
		T weight_ = T(1000);
		// Compute dot-product. It should be zero, for perpendicular vectors
		residuals[0] = T(5)*((X2[0]-X1[0])*(X3[0]-X2[0]) + (X2[1]-X1[1])*(X3[1]-X2[1]) + \
			(X2[2]-X1[2])*(X3[2]-X2[2]));

		return true;
	}

};


// Cost function to store reprojection error resulting from the values of lambdas
struct LambdaReprojectionError{

	// Constructor
	LambdaReprojectionError(double *X, double *x, double *v, double *K, double w, double *trans) \
		: X_(X), x_(x), v_(v), K_(K), w_(w), trans_(trans) {}

	// Operator method. Evaluates the cost function and computes Jacobians.
	template <typename T>
	bool operator() (const T* const rot_, const T* const l, T* residuals) const {

		// 3D wireframe (before applying rotation and translation)
		T P_[3];
		// Initialize the 3D point
		P_[0] = T(X_[0]) + T(l[0])*T(v_[0]) + T(l[1])*T(v_[3]) + T(l[2])*T(v_[6]) + T(l[3])*T(v_[9]) + T(l[4])*T(v_[12]);
		P_[1] = T(X_[1]) + T(l[0])*T(v_[1]) + T(l[1])*T(v_[4]) + T(l[2])*T(v_[7]) + T(l[3])*T(v_[10]) + T(l[4])*T(v_[13]);
		P_[2] = T(X_[2]) + T(l[0])*T(v_[2]) + T(l[1])*T(v_[5]) + T(l[2])*T(v_[8]) + T(l[3])*T(v_[11]) + T(l[4])*T(v_[14]);

		// Apply the rotation and translation
		ceres::AngleAxisRotatePoint(rot_, P_, P_);
		P_[0] += T(trans_[0]);
		P_[1] += T(trans_[1]);
		P_[2] += T(trans_[2]);

		// Project the obtained 3D point down to the image, using the intrinsics (K)
		T p_[3];
		p_[0] = T(K_[0])*P_[0] + T(K_[1])*P_[1] + T(K_[2])*P_[2];
		p_[1] = T(K_[3])*P_[0] + T(K_[4])*P_[1] + T(K_[5])*P_[2];
		p_[2] = T(K_[6])*P_[0] + T(K_[7])*P_[1] + T(K_[8])*P_[2];

		T px_ = p_[0] / p_[2];
		T py_ = p_[1] / p_[2];

		// Compute the residuals (this one works well)
		residuals[0] = T(1)*sqrt(T(w_))*(px_ - T(x_[0]));
		residuals[1] = T(1)*sqrt(T(w_))*(py_ - T(x_[1]));

		return true;
	}

	// 3D points
	double *X_;
	// 2D observations (keypoints)
	double *x_;
	// Top 5 Eigenvectors of the 'current' 3D point
	double *v_;
	// Intrinsic camera matrix
	double *K_;
	// Weight for the current observation
	double w_;
	// Translation estimate (after PnP)
	double *trans_;

};


// Cost function to store (3D) alignment error resulting from the values of lambdas
// Here, 'X_init' is the actual 3D location we are encouraging the car to lie close to
struct LambdaAlignmentError{

	// Constructor
	LambdaAlignmentError(double *X, double *x, double *v, double *K, double w, double *X_init) \
		: X_(X), x_(x), v_(v), K_(K), w_(w), X_init_(X_init) {}

	// Operator method. Evaluates the cost function and computes Jacobians.
	template <typename T>
	bool operator() (const T* const l, T* residuals) const {

		// Temporary variable to hold the 3D keypoint
		T P_[3];
		// Initialize the 3D point
		P_[0] = T(X_[0]) + T(l[0])*T(v_[0]) + T(l[1])*T(v_[3]) + T(l[2])*T(v_[6]) + T(l[3])*T(v_[9]) + T(l[4])*T(v_[12]);
		P_[1] = T(X_[1]) + T(l[0])*T(v_[1]) + T(l[1])*T(v_[4]) + T(l[2])*T(v_[7]) + T(l[3])*T(v_[10]) + T(l[4])*T(v_[13]);
		P_[2] = T(X_[2]) + T(l[0])*T(v_[2]) + T(l[1])*T(v_[5]) + T(l[2])*T(v_[8]) + T(l[3])*T(v_[11]) + T(l[4])*T(v_[14]);

		residuals[0] = T(sqrt(w_))*(T(P_[0]) - T(X_init_[0]));
		residuals[1] = T(sqrt(w_))*(T(P_[0]) - T(X_init_[1]));
		residuals[2] = T(sqrt(w_))*(T(P_[0]) - T(X_init_[2]));

		return true;
	}

	// 3D points
	double *X_;
	// 2D observations (keypoints)
	double *x_;
	// Top 5 Eigenvectors of the 'current' 3D point
	double *v_;
	// Intrinsic camera matrix
	double *K_;
	// Weight for the current observation
	double w_;

	// Initial 3D guess (the value to which we're 'soft' regularizing)
	double *X_init_;

};


// Cost function to prevent lambdas from deforming the shape strongly
struct LambdaRegularizer{

	// Constructor
	LambdaRegularizer(double *v) : v_(v) {}

	// Operator method. Evaluates the cost function and computes the Jacobians.
	template <typename T>
	bool operator() (const T* l, T* residuals) const{

		residuals[0] = 10.0*( l[0]*T(v_[0]) + l[1]*T(v_[3]) + l[2]*T(v_[6]) + l[3]*T(v_[9]) + l[4]*T(v_[12]) );
		residuals[1] = 10.0*( l[0]*T(v_[1]) + l[1]*T(v_[4]) + l[2]*T(v_[7]) + l[3]*T(v_[10]) + l[4]*T(v_[13]) );
		residuals[2] = 10.0*( l[0]*T(v_[2]) + l[1]*T(v_[5]) + l[2]*T(v_[8]) + l[3]*T(v_[11]) + l[4]*T(v_[14]) );

		return true;
	}

	// Top 5 eigenvectors for the 'current' 3D point
	double *v_;

};


// Cost function to prevent lambdas from deforming the shape strongly
// Here, each of the lambdas has a different weight
struct LambdaRegularizerWeighted{

	// Constructor
	LambdaRegularizerWeighted(double *w) : w_(w) {}

	// Operator method. Evaluates the cost function and computes the Jacobians.
	template <typename T>
	bool operator() (const T* l, T* residuals) const{

		residuals[0] = T(sqrt(w_[0]))*l[0];
		residuals[1] = T(sqrt(w_[1]))*l[1];
		residuals[2] = T(sqrt(w_[2]))*l[2];
		residuals[3] = T(sqrt(w_[3]))*l[3];
		residuals[4] = T(sqrt(w_[4]))*l[4];

		return true;
	}

	// Weight to be applied to each lambda
	double *w_;

};


// Cost function to regularize the translation estimate, to prevent a huge drift from the initial estimate
struct TranslationRegularizer{

	// Constructor
	TranslationRegularizer(double *trans_init) : trans_init_(trans_init) {}

	// Operator method. Evaluates the cost function and computes the Jacobians.
	template <typename T>
	bool operator() (const T* trans, T* residuals) const{

		residuals[0] = T(10)*trans[0];
		residuals[1] = T(10)*trans[1];
		residuals[2] = T(10)*trans[2];

		return true;
	}

	// Initial translation estimate
	double *trans_init_;

};


// Cost function to regularize the rotation estimate, to align the axis of rotation with the Y-axix
struct RotationRegularizer{

	// Constructor
	RotationRegularizer() {}

	// Operator method. Evaluates the cost function and computes the Jacobians.
	template <typename T>
	bool operator() (const T* rot, T* residuals) const{

		residuals[0] = T(5000)*rot[0];
		residuals[1] = T(0);
		residuals[2] = T(5000)*rot[2];

		return true;
	}

};


// Cost function to optimize over R,t, given the 3D and 2D keypoints, as in PnP
struct PnPError{

	// Constructor
	PnPError(double *X, double *x, double *v, double *K, double w, double *l) \
		: X_(X), x_(x), v_(v), K_(K), w_(w), l_(l) {}

	// Operator method. Evaluates the cost function and computes Jacobians.
	template <typename T>
	bool operator() (const T* const rot, const T* trans, T* residuals) const {

		// Temporary variable to hold the 3D keypoint
		T P_[3];
		// Initialize the 3D point
		P_[0] = T(X_[0]) + T(l_[0])*T(v_[0]) + T(l_[1])*T(v_[3]) + T(l_[2])*T(v_[6]) + T(l_[3])*T(v_[9]) + T(l_[4])*T(v_[12]);
		P_[1] = T(X_[1]) + T(l_[0])*T(v_[1]) + T(l_[1])*T(v_[4]) + T(l_[2])*T(v_[7]) + T(l_[3])*T(v_[10]) + T(l_[4])*T(v_[13]);
		P_[2] = T(X_[2]) + T(l_[0])*T(v_[2]) + T(l_[1])*T(v_[5]) + T(l_[2])*T(v_[8]) + T(l_[3])*T(v_[11]) + T(l_[4])*T(v_[14]);

		// Rotate the point (and store the result in the same variable)
		// Order of arguments passed: (axis-angle rotation vector (size 3), point (size 3), array where result is to be stored (size 3))
		ceres::AngleAxisRotatePoint(rot, P_, P_);
		// Add the translation
		P_[0] = T(P_[0]) + trans[0];
		P_[1] = T(P_[1]) + trans[1];
		P_[2] = T(P_[2]) + trans[2];

		// Project the obtained 3D point down to the image, using the intrinsics (K)
		T p_[3];
		p_[0] = T(K_[0])*P_[0] + T(K_[1])*P_[1] + T(K_[2])*P_[2];
		p_[1] = T(K_[3])*P_[0] + T(K_[4])*P_[1] + T(K_[5])*P_[2];
		p_[2] = T(K_[6])*P_[0] + T(K_[7])*P_[1] + T(K_[8])*P_[2];


		// T p[3];
		// // T P_X = l[0]*T(v_[0]) + l[1]*T(v_[3]) + l[2]*T(v_[6]) + l[3]*T(v_[9]) + l[4]*T(v_[12]);
		// // T P_Y = l[0]*T(v_[1]) + l[1]*T(v_[4]) + l[2]*T(v_[7]) + l[3]*T(v_[10]) + l[4]*T(v_[13]);
		// // T P_Z = l[0]*T(v_[2]) + l[1]*T(v_[5]) + l[2]*T(v_[8]) + l[3]*T(v_[11]) + l[4]*T(v_[14]);
		// p[0] = T(K_[0])*(T(X_[0]) + T(l_[0])*T(v_[0]) + T(l_[1])*T(v_[3]) + T(l_[2])*T(v_[6]) + T(l_[3])*T(v_[9]) + T(l_[4])*T(v_[12]) )   +   T(K_[1])*(T(X_[1]) + T(l_[0])*T(v_[1]) + T(l_[1])*T(v_[4]) + T(l_[2])*T(v_[7]) + T(l_[3])*T(v_[10]) + T(l_[4])*T(v_[13]) )   +   T(K_[2])*(T(X_[2]) + T(l_[0])*T(v_[2]) + T(l_[1])*T(v_[5]) + T(l_[2])*T(v_[8]) + T(l_[3])*T(v_[11]) + T(l_[4])*T(v_[14]) );
		// p[1] = T(K_[3])*(T(X_[0]) + T(l_[0])*T(v_[0]) + T(l_[1])*T(v_[3]) + T(l_[2])*T(v_[6]) + T(l_[3])*T(v_[9]) + T(l_[4])*T(v_[12]) )   +   T(K_[4])*(T(X_[1]) + T(l_[0])*T(v_[1]) + T(l_[1])*T(v_[4]) + T(l_[2])*T(v_[7]) + T(l_[3])*T(v_[10]) + T(l_[4])*T(v_[13]) )   +   T(K_[5])*(T(X_[2]) + T(l_[0])*T(v_[2]) + T(l_[1])*T(v_[5]) + T(l_[2])*T(v_[8]) + T(l_[3])*T(v_[11]) + T(l_[4])*T(v_[14]) );
		// p[2] = T(K_[6])*(T(X_[0]) + T(l_[0])*T(v_[0]) + T(l_[1])*T(v_[3]) + T(l_[2])*T(v_[6]) + T(l_[3])*T(v_[9]) + T(l_[4])*T(v_[12]) )   +   T(K_[7])*(T(X_[1]) + T(l_[0])*T(v_[1]) + T(l_[1])*T(v_[4]) + T(l_[2])*T(v_[7]) + T(l_[3])*T(v_[10]) + T(l_[4])*T(v_[13]) )   +   T(K_[8])*(T(X_[2]) + T(l_[0])*T(v_[2]) + T(l_[1])*T(v_[5]) + T(l_[2])*T(v_[8]) + T(l_[3])*T(v_[11]) + T(l_[4])*T(v_[14]) );

		T px_ = p_[0] / p_[2];
		T py_ = p_[1] / p_[2];

		// Compute the residuals (this one works well)
		residuals[0] = T(1)*sqrt(T(w_))*(px_ - T(x_[0]));
		residuals[1] = T(1)*sqrt(T(w_))*(py_ - T(x_[1]));

		// residuals[0] = sqrt(T(w_))*(((T(K_[0])*(T(X_[0]) + T(l_[0])*T(v_[0]) + T(l_[1])*T(v_[3]) + T(l_[2])*T(v_[6]) + T(l_[3])*T(v_[9]) + T(l_[4])*T(v_[12]) )   +   T(K_[1])*(T(X_[1]) + T(l_[0])*T(v_[1]) + T(l_[1])*T(v_[4]) + T(l_[2])*T(v_[7]) + T(l_[3])*T(v_[10]) + T(l_[4])*T(v_[13]) )   +   T(K_[2])*(T(X_[2]) + T(l_[0])*T(v_[2]) + T(l_[1])*T(v_[5]) + T(l_[2])*T(v_[8]) + T(l_[3])*T(v_[11]) + T(l_[4])*T(v_[14]) )) / (T(K_[6])*(T(X_[0]) + T(l_[0])*T(v_[0]) + T(l_[1])*T(v_[3]) + T(l_[2])*T(v_[6]) + T(l_[3])*T(v_[9]) + T(l_[4])*T(v_[12]) )   +   T(K_[7])*(T(X_[1]) + T(l_[0])*T(v_[1]) + T(l_[1])*T(v_[4]) + T(l_[2])*T(v_[7]) + T(l_[3])*T(v_[10]) + T(l_[4])*T(v_[13]) )   +   T(K_[8])*(T(X_[2]) + T(l_[0])*T(v_[2]) + T(l_[1])*T(v_[5]) + T(l_[2])*T(v_[8]) + T(l_[3])*T(v_[11]) + T(l_[4])*T(v_[14]) ))) - T(x_[0]));
		// residuals[1] = sqrt(T(w_))*(((T(K_[3])*(T(X_[0]) + T(l_[0])*T(v_[0]) + T(l_[1])*T(v_[3]) + T(l_[2])*T(v_[6]) + T(l_[3])*T(v_[9]) + T(l_[4])*T(v_[12]) )   +   T(K_[4])*(T(X_[1]) + T(l_[0])*T(v_[1]) + T(l_[1])*T(v_[4]) + T(l_[2])*T(v_[7]) + T(l_[3])*T(v_[10]) + T(l_[4])*T(v_[13]) )   +   T(K_[5])*(T(X_[2]) + T(l_[0])*T(v_[2]) + T(l_[1])*T(v_[5]) + T(l_[2])*T(v_[8]) + T(l_[3])*T(v_[11]) + T(l_[4])*T(v_[14]) )) / (T(K_[6])*(T(X_[0]) + T(l_[0])*T(v_[0]) + T(l_[1])*T(v_[3]) + T(l_[2])*T(v_[6]) + T(l_[3])*T(v_[9]) + T(l_[4])*T(v_[12]) )   +   T(K_[7])*(T(X_[1]) + T(l_[0])*T(v_[1]) + T(l_[1])*T(v_[4]) + T(l_[2])*T(v_[7]) + T(l_[3])*T(v_[10]) + T(l_[4])*T(v_[13]) )   +   T(K_[8])*(T(X_[2]) + T(l_[0])*T(v_[2]) + T(l_[1])*T(v_[5]) + T(l_[2])*T(v_[8]) + T(l_[3])*T(v_[11]) + T(l_[4])*T(v_[14]) ))) - T(x_[1]));

		return true;
	}

	// 3D points
	double *X_;
	// 2D observations (keypoints)
	double *x_;
	// Top 5 Eigenvectors of the 'current' 3D point
	double *v_;
	// Intrinsic camera matrix
	double *K_;
	// Weight for the current observation
	double w_;
	// Weights for each shape basis vector (lambdas)
	double *l_;

};


// Storing old (commented) versions of error terms here (for reference)

// // Cost function to store reprojection error resulting from the values of lambdas
// struct LambdaReprojectionError{

// 	// Constructor
// 	LambdaReprojectionError(double *X, double *x, double *v, double *K, double w, double *rot, double *trans) \
// 		: X_(X), x_(x), v_(v), K_(K), w_(w), rot_(rot), trans_(trans) {}

// 	// Operator method. Evaluates the cost function and computes Jacobians.
// 	template <typename T>
// 	bool operator() (const T* const l, T* residuals) const {

// 		T p[3];
// 		// T P_X = l[0]*T(v_[0]) + l[1]*T(v_[3]) + l[2]*T(v_[6]) + l[3]*T(v_[9]) + l[4]*T(v_[12]);
// 		// T P_Y = l[0]*T(v_[1]) + l[1]*T(v_[4]) + l[2]*T(v_[7]) + l[3]*T(v_[10]) + l[4]*T(v_[13]);
// 		// T P_Z = l[0]*T(v_[2]) + l[1]*T(v_[5]) + l[2]*T(v_[8]) + l[3]*T(v_[11]) + l[4]*T(v_[14]);
// 		p[0] = T(K_[0])*(T(X_[0]) + l[0]*T(v_[0]) + l[1]*T(v_[3]) + l[2]*T(v_[6]) + l[3]*T(v_[9]) + l[4]*T(v_[12]) )   +   T(K_[1])*(T(X_[1]) + l[0]*T(v_[1]) + l[1]*T(v_[4]) + l[2]*T(v_[7]) + l[3]*T(v_[10]) + l[4]*T(v_[13]) )   +   T(K_[2])*(T(X_[2]) + l[0]*T(v_[2]) + l[1]*T(v_[5]) + l[2]*T(v_[8]) + l[3]*T(v_[11]) + l[4]*T(v_[14]) );
// 		p[1] = T(K_[3])*(T(X_[0]) + l[0]*T(v_[0]) + l[1]*T(v_[3]) + l[2]*T(v_[6]) + l[3]*T(v_[9]) + l[4]*T(v_[12]) )   +   T(K_[4])*(T(X_[1]) + l[0]*T(v_[1]) + l[1]*T(v_[4]) + l[2]*T(v_[7]) + l[3]*T(v_[10]) + l[4]*T(v_[13]) )   +   T(K_[5])*(T(X_[2]) + l[0]*T(v_[2]) + l[1]*T(v_[5]) + l[2]*T(v_[8]) + l[3]*T(v_[11]) + l[4]*T(v_[14]) );
// 		p[2] = T(K_[6])*(T(X_[0]) + l[0]*T(v_[0]) + l[1]*T(v_[3]) + l[2]*T(v_[6]) + l[3]*T(v_[9]) + l[4]*T(v_[12]) )   +   T(K_[7])*(T(X_[1]) + l[0]*T(v_[1]) + l[1]*T(v_[4]) + l[2]*T(v_[7]) + l[3]*T(v_[10]) + l[4]*T(v_[13]) )   +   T(K_[8])*(T(X_[2]) + l[0]*T(v_[2]) + l[1]*T(v_[5]) + l[2]*T(v_[8]) + l[3]*T(v_[11]) + l[4]*T(v_[14]) );

// 		T px = p[0] / p[2];
// 		T py = p[1] / p[2];

// 		// // Compute the residuals (this one works well)
// 		// residuals[0] = px - T(x_[0]);
// 		// residuals[1] = py - T(x_[1]);

// 		residuals[0] = T(sqrt(w_))*(((T(K_[0])*(T(X_[0]) + l[0]*T(v_[0]) + l[1]*T(v_[3]) + l[2]*T(v_[6]) + l[3]*T(v_[9]) + l[4]*T(v_[12]) )   +   T(K_[1])*(T(X_[1]) + l[0]*T(v_[1]) + l[1]*T(v_[4]) + l[2]*T(v_[7]) + l[3]*T(v_[10]) + l[4]*T(v_[13]) )   +   T(K_[2])*(T(X_[2]) + l[0]*T(v_[2]) + l[1]*T(v_[5]) + l[2]*T(v_[8]) + l[3]*T(v_[11]) + l[4]*T(v_[14]) )) / (T(K_[6])*(T(X_[0]) + l[0]*T(v_[0]) + l[1]*T(v_[3]) + l[2]*T(v_[6]) + l[3]*T(v_[9]) + l[4]*T(v_[12]) )   +   T(K_[7])*(T(X_[1]) + l[0]*T(v_[1]) + l[1]*T(v_[4]) + l[2]*T(v_[7]) + l[3]*T(v_[10]) + l[4]*T(v_[13]) )   +   T(K_[8])*(T(X_[2]) + l[0]*T(v_[2]) + l[1]*T(v_[5]) + l[2]*T(v_[8]) + l[3]*T(v_[11]) + l[4]*T(v_[14]) ))) - T(x_[0]));
// 		residuals[1] = T(sqrt(w_))*(((T(K_[3])*(T(X_[0]) + l[0]*T(v_[0]) + l[1]*T(v_[3]) + l[2]*T(v_[6]) + l[3]*T(v_[9]) + l[4]*T(v_[12]) )   +   T(K_[4])*(T(X_[1]) + l[0]*T(v_[1]) + l[1]*T(v_[4]) + l[2]*T(v_[7]) + l[3]*T(v_[10]) + l[4]*T(v_[13]) )   +   T(K_[5])*(T(X_[2]) + l[0]*T(v_[2]) + l[1]*T(v_[5]) + l[2]*T(v_[8]) + l[3]*T(v_[11]) + l[4]*T(v_[14]) )) / (T(K_[6])*(T(X_[0]) + l[0]*T(v_[0]) + l[1]*T(v_[3]) + l[2]*T(v_[6]) + l[3]*T(v_[9]) + l[4]*T(v_[12]) )   +   T(K_[7])*(T(X_[1]) + l[0]*T(v_[1]) + l[1]*T(v_[4]) + l[2]*T(v_[7]) + l[3]*T(v_[10]) + l[4]*T(v_[13]) )   +   T(K_[8])*(T(X_[2]) + l[0]*T(v_[2]) + l[1]*T(v_[5]) + l[2]*T(v_[8]) + l[3]*T(v_[11]) + l[4]*T(v_[14]) ))) - T(x_[1]));

// 		return true;
// 	}

// 	// 3D points
// 	double *X_;
// 	// 2D observations (keypoints)
// 	double *x_;
// 	// Top 5 Eigenvectors of the 'current' 3D point
// 	double *v_;
// 	// Intrinsic camera matrix
// 	double *K_;
// 	// Weight for the current observation
// 	double w_;
// 	// Rotation estimate (after PnP)
// 	double rot_;
// 	// Translation estimate (after PnP)
// 	double trans_;

// };
