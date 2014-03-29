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

#include "changepoint/articulation.h"

namespace changepoint{

    
std::string ArticulationParams::getModelName(){
    return params->name;
}

double ArticulationParams::getParam(std::string name) 
{
    for(size_t i=0;i<params->params.size();i++) {
            if(params->params[i].name == name) {
                return(params->params[i].value);
            }
    }
    return 0.00;
}


void ArticulationParams::printParams()
{
    if(params->name == "rigid"){
        printf("Rigid Model:\n");
        printf("    position.x: %f\n", getParam("rigid_position.x"));
        printf("    position.y: %f\n", getParam("rigid_position.y"));
        printf("    position.z: %f\n", getParam("rigid_position.z"));
        printf("    orientation.x: %f\n", getParam("rigid_orientation.x"));
        printf("    orientation.y: %f\n", getParam("rigid_orientation.y"));
        printf("    orientation.z: %f\n", getParam("rigid_orientation.z"));
        printf("    orientation.w: %f\n", getParam("rigid_orientation.w"));
        printf("Log Likelihood: %f\n",logLikelihood);
        printf("Model Evidence: %f\n",modelEvidence);
    }
    else if(params->name == "prismatic"){
        printf("Prismatic Model:\n");
        printf("    prismatic_dir.x: %f\n", getParam("prismatic_dir.x"));
        printf("    prismatic_dir.y: %f\n", getParam("prismatic_dir.y"));
        printf("    prismatic_dir.z: %f\n", getParam("prismatic_dir.z"));
        printf("    position.x: %f\n", getParam("rigid_position.x"));
        printf("    position.y: %f\n", getParam("rigid_position.y"));
        printf("    position.z: %f\n", getParam("rigid_position.z"));
        printf("    orientation.x: %f\n", getParam("rigid_orientation.x"));
        printf("    orientation.y: %f\n", getParam("rigid_orientation.y"));
        printf("    orientation.z: %f\n", getParam("rigid_orientation.z"));
        printf("    orientation.w: %f\n", getParam("rigid_orientation.w"));
        printf("Log Likelihood: %f\n",logLikelihood);
        printf("Model Evidence: %f\n",modelEvidence);
    }
    else if(params->name == "rotational"){
        printf("Rotational Model:\n");
        printf("    radius: %f\n", getParam("rot_radius"));
        printf("    center.x: %f\n", getParam("rot_center.x"));
        printf("    center.y: %f\n", getParam("rot_center.y"));
        printf("    center.z: %f\n", getParam("rot_center.z"));
        printf("    axis.x: %f\n", getParam("rot_axis.x"));
        printf("    axis.y: %f\n", getParam("rot_axis.y"));
        printf("    axis.z: %f\n", getParam("rot_axis.z"));
        printf("    axis.w: %f\n", getParam("rot_axis.w"));
        printf("    orientation.x: %f\n", getParam("rot_orientation.x"));
        printf("    orientation.y: %f\n", getParam("rot_orientation.y"));
        printf("    orientation.z: %f\n", getParam("rot_orientation.z"));
        printf("    orientation.w: %f\n", getParam("rot_orientation.w"));
        printf("Log Likelihood: %f\n",logLikelihood);
        printf("Model Evidence: %f\n",modelEvidence);
    }
    printf("\n");
}


void ArticulationParams::fillParams(ModelSegment &seg)
{
    if(params->name == "rigid"){
        seg.param_names.push_back("rigid_position.x");
        seg.param_names.push_back("rigid_position.y");
        seg.param_names.push_back("rigid_position.z");
        seg.param_names.push_back("rigid_orientation.x");
        seg.param_names.push_back("rigid_orientation.y");
        seg.param_names.push_back("rigid_orientation.z");
        seg.param_names.push_back("rigid_orientation.w");
        seg.param_names.push_back("log_likelihood");
        seg.param_names.push_back("model_evidence");
        
        seg.model_params.push_back(getParam("rigid_position.x"));
        seg.model_params.push_back(getParam("rigid_position.y"));
        seg.model_params.push_back(getParam("rigid_position.z"));
        seg.model_params.push_back(getParam("rigid_orientation.x"));
        seg.model_params.push_back(getParam("rigid_orientation.y"));
        seg.model_params.push_back(getParam("rigid_orientation.z"));
        seg.model_params.push_back(getParam("rigid_orientation.w"));
        seg.model_params.push_back(logLikelihood);
        seg.model_params.push_back(modelEvidence);
    }
    
    else if(params->name == "prismatic"){
        seg.param_names.push_back("prismatic_dir.x");
        seg.param_names.push_back("prismatic_dir.y");
        seg.param_names.push_back("prismatic_dir.z");
        seg.param_names.push_back("rigid_position.x");
        seg.param_names.push_back("rigid_position.y");
        seg.param_names.push_back("rigid_position.z");
        seg.param_names.push_back("rigid_orientation.x");
        seg.param_names.push_back("rigid_orientation.y");
        seg.param_names.push_back("rigid_orientation.z");
        seg.param_names.push_back("rigid_orientation.w");
        seg.param_names.push_back("log_likelihood");
        seg.param_names.push_back("model_evidence");
        
        seg.model_params.push_back(getParam("prismatic_dir.x"));
        seg.model_params.push_back(getParam("prismatic_dir.y"));
        seg.model_params.push_back(getParam("prismatic_dir.z"));
        seg.model_params.push_back(getParam("rigid_position.x"));
        seg.model_params.push_back(getParam("rigid_position.y"));
        seg.model_params.push_back(getParam("rigid_position.z"));
        seg.model_params.push_back(getParam("rigid_orientation.x"));
        seg.model_params.push_back(getParam("rigid_orientation.y"));
        seg.model_params.push_back(getParam("rigid_orientation.z"));
        seg.model_params.push_back(getParam("rigid_orientation.w"));
        seg.model_params.push_back(logLikelihood);
        seg.model_params.push_back(modelEvidence);
    }
    
    else if(params->name == "rotational"){
        seg.param_names.push_back("rot_radius");
        seg.param_names.push_back("rot_center.x");
        seg.param_names.push_back("rot_center.y");
        seg.param_names.push_back("rot_center.z");
        seg.param_names.push_back("rot_axis.x");
        seg.param_names.push_back("rot_axis.y");
        seg.param_names.push_back("rot_axis.z");
        seg.param_names.push_back("rot_axis.w");
        seg.param_names.push_back("rot_orientation.x");
        seg.param_names.push_back("rot_orientation.y");
        seg.param_names.push_back("rot_orientation.z");
        seg.param_names.push_back("rot_orientation.w");
        seg.param_names.push_back("log_likelihood");
        seg.param_names.push_back("model_evidence");
        
        seg.model_params.push_back(getParam("rot_radius"));
        seg.model_params.push_back(getParam("rot_center.x"));
        seg.model_params.push_back(getParam("rot_center.y"));
        seg.model_params.push_back(getParam("rot_center.z"));
        seg.model_params.push_back(getParam("rot_axis.x"));
        seg.model_params.push_back(getParam("rot_axis.y"));
        seg.model_params.push_back(getParam("rot_axis.z"));
        seg.model_params.push_back(getParam("rot_axis.w"));
        seg.model_params.push_back(getParam("rot_orientation.x"));
        seg.model_params.push_back(getParam("rot_orientation.y"));
        
        seg.model_params.push_back(getParam("rot_orientation.z"));
        seg.model_params.push_back(getParam("rot_orientation.w"));
        seg.model_params.push_back(logLikelihood);
        seg.model_params.push_back(modelEvidence);
    }
}
    
//*****************************************************************************************************    
    
    
ArticulationFitter::ArticulationFitter(int model_id)
{
    double sigma_position = 0.0075;
    double sigma_orientation = M_PI/10.0;  
    double optimizer_iterations = 0;
    double sac_iterations = 50;
    double prior_outlier_ratio = log(0.01) / (- 0.05); 
    
    std::string name;
    if(model_id == 0){
        name = "rigid";
        gm = boost::shared_ptr<articulation_models::GenericModel>(new articulation_models::RigidModel());
    }
    else if(model_id == 1){
        name = "prismatic";
        gm = boost::shared_ptr<articulation_models::GenericModel>(new articulation_models::PrismaticModel());    
    }
    else if(model_id == 2){
        name = "rotational";
        gm = boost::shared_ptr<articulation_models::GenericModel>(new articulation_models::RotationalModel());
    }
    else{
        std::cout << "ModelFitter: Non-recognized model type " << model_id << "\n";
        return;
    }
    
    gm->model.name = name.c_str();
    gm->sigma_position = sigma_position;
    gm->sigma_orientation = sigma_orientation;
    gm->optimizer_iterations = optimizer_iterations;
    gm->sac_iterations = sac_iterations;
    gm->prior_outlier_ratio = prior_outlier_ratio;
}


bool ArticulationFitter::fitSegment(double **data, const int start, const int end, ModelParams *mp)
{   
    ArticulationParams *ap = static_cast<ArticulationParams*>(mp);
    
    // Load trajectory segment into a track message
    articulation_msgs::TrackMsg track;
    for(int i=start+1; i<=end; i++){
        geometry_msgs::Pose temp;
        temp.position.x = data[i][0];
        temp.position.y = data[i][1];
        temp.position.z = data[i][2];
        temp.orientation.x = data[i][3];
        temp.orientation.y = data[i][4];
        temp.orientation.z = data[i][5];
        temp.orientation.w = data[i][6];
        track.pose.push_back(temp);
    }
    gm->setTrack(track);
    
    // fit model, check for validity
    if (!gm->fitModel()) {
        ROS_INFO_STREAM( ap->params->name << ": fitting failed" );
        return false;
    }        
    if (!gm->evaluateModel()) {
        ROS_INFO_STREAM( ap->params->name << ": eval failed" );
        return false;
    }
    
    ap->params = &(gm->model);
    ap->logLikelihood = gm->getLogLikelihood(true);
    ap->modelEvidence = (-(gm->getBIC())/ 2.0); //Convert to form in Bishop that approximates model evidence
      
    return true;
}

} // end namespace
