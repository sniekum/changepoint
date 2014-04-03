/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Scott Niekum
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/**
  * \author Scott Niekum
  */

#include "changepoint/gauss1D.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(changepoint, Gauss1DFitter, changepoint::Gauss1DFitter, changepoint::ModelFitter)


namespace changepoint{

 
std::string Gauss1DParams::getModelName(){
    return "Gaussian";
}

void Gauss1DParams::printParams()
{
    printf("Mu: %f\n", mu);
    printf("Sigma: %f\n", sigma);
    printf("Log Likelihood: %f\n",logLikelihood);
    printf("Model Evidence: %f\n\n",modelEvidence);
}
    
void Gauss1DParams::fillParams(ModelSegment &seg)
{
    seg.param_names.push_back("mu");
    seg.param_names.push_back("sigma");
    seg.param_names.push_back("log_likelihood");
    seg.param_names.push_back("model_evidence");
    
    seg.model_params.push_back(mu);
    seg.model_params.push_back(sigma);
    seg.model_params.push_back(logLikelihood);
    seg.model_params.push_back(modelEvidence);
}
    
//***************************************************************************************************************    
    
    
bool Gauss1DFitter::fitSegment(double **data, const int start, const int end)
{    
    Gauss1DParams *gp = static_cast<Gauss1DParams*>(mp);
    
    int n = end-start;
    
    double mu = 0;
    for(int i=start+1; i<=end; i++)
        mu += data[i][0];
    mu /= n;
    
    double sigma = 0;
    for(int i=start+1; i<=end; i++)
        sigma += pow(data[i][0]-mu,2);
    sigma = sqrt(sigma/n);
    
    gp->mu = mu;
    gp->sigma = sigma;  
    
    double diff = 0;
    for(int i=start+1; i<=end; i++)
        diff += pow(data[i][0]-gp->mu,2);
    
    double term1 = (-n/2.0) * log(2.0*M_PI);
    double term2 = (-n/2.0) * log(gp->sigma*gp->sigma);
    double term3 = -(diff/(2*gp->sigma*gp->sigma));
    
    gp->logLikelihood = term1 + term2 + term3;
    gp->modelEvidence = gp->logLikelihood - log(n);  // LL - 1/2 (2 ln n)
    
    return true;
}

} // end namespace