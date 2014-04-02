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

#ifndef GAUSS1D_H_
#define GAUSS1D_H_

#include "changepoint/cp_detector.h"

namespace changepoint{

class Gauss1DParams : public ModelParams
{
public:
    Gauss1DParams(){};
    Gauss1DParams(double m, double s){mu=m; sigma=s;}
    ~Gauss1DParams(){};
    
    Gauss1DParams(ModelParams *rhs){
        Gauss1DParams *gp = static_cast<Gauss1DParams*>(rhs);
        mu = gp->mu; 
        sigma = gp->sigma;
        logLikelihood = gp->logLikelihood;
        modelEvidence = gp->modelEvidence;
    }
    
    std::string getModelName();
    void printParams();
    void fillParams(ModelSegment &seg);
    std::vector< std::vector<double> > calcFinalSegStats(double **data, const int start, const int end)
    {
        std::vector< std::vector<double> > empty;
        return empty;
    };

    double mu, sigma;
};


class Gauss1DFitter : public ModelFitter{
public:
    Gauss1DFitter(int model_id){mp = new Gauss1DParams();}
    ~Gauss1DFitter(){delete mp;}
    Gauss1DFitter(ModelFitter *rhs){
        Gauss1DFitter *gf = static_cast<Gauss1DFitter*>(rhs);
        mp = new Gauss1DParams(gf->mp);
    }
    
    bool fitSegment(double **data, const int start, const int end);
    int nModels(){return 1;}

};

} // end namespace

#endif //GAUSS1D_H_