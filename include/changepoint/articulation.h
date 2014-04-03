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

#ifndef ARTICULATION_H_
#define ARTICULATION_H_

#include "changepoint/cp_detector.h"
#include "articulation_models/models/factory.h"
#include "articulation_models/models/generic_model.h"
#include "articulation_models/models/rigid_model.h"
#include "articulation_models/models/prismatic_model.h"
#include "articulation_models/models/rotational_model.h"
#include "articulation_msgs/ModelMsg.h"
#include "articulation_msgs/TrackMsg.h"
#include "articulation_msgs/ParamMsg.h"
#include "articulation_msgs/TrackModelSrv.h"
#include "articulation_models/utils.h"

namespace changepoint{

class ArticulationParams : public ModelParams
{
public:
    ArticulationParams(){params = new articulation_msgs::ModelMsg();}
    ~ArticulationParams(){delete params;}
    
    ArticulationParams(ModelParams *rhs){
        ArticulationParams *ap = static_cast<ArticulationParams*>(rhs);
        params = new articulation_msgs::ModelMsg(*(ap->params));
        logLikelihood = ap->logLikelihood;
        modelEvidence = ap->modelEvidence;
    }
    
    std::string getModelName();
    void printParams();
    void fillParams(ModelSegment &seg);
    
    articulation_msgs::ModelMsg *params;
    
private:
    double getParam(std::string name);
};


class ArticulationFitter : public ModelFitter{
public:
    ArticulationFitter(){};
    ~ArticulationFitter(){if(mp!=NULL) delete mp;}
    
    void initialize(int model_id);
    void copyFrom(ModelFitter *rhs); 
    bool fitSegment(double **data, const int start, const int end);
    int nModels(){return 3;}
    std::vector< std::vector<double> > calcFinalSegStats(double **data, const int start, const int end);

    boost::shared_ptr<articulation_models::GenericModel> gm; 
    int m_id;
};

} // end namespace

#endif //ARTICULATION_H_