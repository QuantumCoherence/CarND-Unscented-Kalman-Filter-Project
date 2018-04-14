#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
			rmse << 0,0,0,0;

		    VectorXd temp(4);
			// check the validity of the following inputs:
			//  * the estimation vector size should not be zero
			if (estimations.size() == 0){
			    cout << "Estimation vector size null!" << endl;
			    return rmse;
			}
			//  * the estimation vector size should be equal ground truth vector size
			if (estimations.size() != ground_truth.size()){
			    cout << "Estimation anb Ground_Truth vector size mismatch!" << endl;
			    return rmse;
			}


			//accumulate squared residuals
			for(int i=0; i < estimations.size(); ++i){
			    temp = estimations[i]-ground_truth[i];
			    temp = temp.array()*temp.array();
		        rmse +=  temp;
			}
		    rmse = rmse.array()/estimations.size();

			//calculate the squared root
			rmse = rmse.array().sqrt();

			//return the result
            return rmse;
}
