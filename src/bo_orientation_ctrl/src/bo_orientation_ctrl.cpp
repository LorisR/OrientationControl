
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <vector>
#include <utility>
#include <math.h>
#include <time.h>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <array>
#include <functional>

#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>
#include <limbo/serialize/text_archive.hpp>

using namespace limbo;

struct Params {
    struct kernel_exp {
        BO_PARAM(double, sigma_sq, 1.0);
        BO_PARAM(double, l, 0.2);
    };
    struct kernel : public defaults::kernel {
    };
    struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
    };
    struct opt_rprop : public defaults::opt_rprop {
    };
};

double rpy_msr1, rpy_msr2, cost, update_cost;

void upcostCallback(const std_msgs::Float64::ConstPtr& msgupcost)
{

	update_cost = msgupcost->data;

}

void costCallback(const std_msgs::Float64::ConstPtr& msgcost)
{

	cost = msgcost->data;

}

void rpy_msr1Callback(const std_msgs::Float64::ConstPtr& msg1)
{

	rpy_msr1 = msg1->data;
	if (rpy_msr1>0.)
		rpy_msr1 = rpy_msr1 - 3.14*2.;
	//std::cout << "rpy_msr1: " << rpy_msr1 << std::endl;

}

void rpy_msr2Callback(const std_msgs::Float64::ConstPtr& msg2)
{

	rpy_msr2 = msg2->data;
	//std::cout << "rpy_msr2: " << rpy_msr2 << std::endl;

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "BOorctrl");
	
	ros::NodeHandle n;
	
	ros::Subscriber rpy_msr1_sub = n.subscribe("/rpy_msr1",20,rpy_msr1Callback);
	ros::Subscriber rpy_msr2_sub = n.subscribe("/rpy_msr2",20,rpy_msr2Callback);
	ros::Subscriber cost_sub = n.subscribe("/cost",20,costCallback);
	ros::Subscriber update_cost_sub = n.subscribe("/update_cost",20,upcostCallback);
	
	ros::Publisher BOsent_pub = n.advertise<std_msgs::Float64>("/rpy_gp_update",20);
	ros::Publisher cmd_rot_1_pub = n.advertise<std_msgs::Float64>("/rpy_gp_1",20);
	ros::Publisher cmd_rot_2_pub = n.advertise<std_msgs::Float64>("/rpy_gp_2",20);
	
	ros::Rate loop_rate(10);
	
	std::vector<Eigen::VectorXd> samples;
    	std::vector<Eigen::VectorXd> observations;
	
	int cont = 0;
	Eigen::VectorXd data1;
	Eigen::VectorXd data2;
	
	bool update_gp_done = false;
	
	while (ros::ok())
	{
	
		std_msgs::Float64 rpy_gp_update;
		rpy_gp_update.data = 0.;
		
		if (update_cost>1)
		{
			// std::cout << "dentro l'if" << std::endl;
			
			cont++;
			
			Eigen::VectorXd current_sample(2,1);
			current_sample[0] = rpy_msr1;
			current_sample[1] = rpy_msr2;
			
			samples.push_back(current_sample);
			observations.push_back(tools::make_vector(cost));
			
			// std::cout << samples[cont-1] << std::endl;
			// std::cout << "" << std::endl;
			// std::cout << current_sample << std::endl;
			// std::cout << "" << std::endl;
			// std::cout << observations[cont-1] << std::endl;
			// std::cout << "******" << std::endl;
			
			using Kernel_t = kernel::Exp<Params>;
    			using Mean_t = mean::Data<Params>;
    			using GP_t = model::GP<Params, Kernel_t, Mean_t>;

    			// 2-D inputs, 1-D outputs
    			GP_t gp(2, 1);
			
			gp.compute(samples, observations);
			
			double mu_min = 1000000.;
			
			Eigen::VectorXd ranges(2,1);
			
			std_msgs::Float64 rpy_gp_1;
			std_msgs::Float64 rpy_gp_2;
			
			update_gp_done = false;
			
			if (cont % 20 == 0 && cont>100)
			{
			
				int index_i = 0;
				int index_j = 0;
				double sigma_min = 0.;
			
				for (int i = 0; i < 60; i++) {
					for (int j = 0; j < 60; j++) {
					
						ranges[0] = (-30.+i-180.)*3.14/180.;
						ranges[1] = (-30.+j)*3.14/180.;
					
						Eigen::VectorXd mu;
						double sigma;
						
						std::tie(mu, sigma) = gp.query(ranges);
						
						//std::cout << "mu[0]" << mu[0] << std::endl;
				
						if (sigma < 0.65 && mu[0] < mu_min && mu[0]>0.)
						{
							mu_min = mu[0];
							sigma_min = sigma;
							index_i = i;
							index_j = j;
							update_gp_done = true;
							
						}	
					}
	    			}
	    			
	    			if (update_gp_done)
	    			{
	    				rpy_gp_1.data = (-30.+index_i-180.)*3.14/180.;
					rpy_gp_2.data = (-30.+index_j)*3.14/180.;
					std::cout << "pubblico aggiornamento setpoint rotazioni" << std::endl;
					cmd_rot_1_pub.publish(rpy_gp_1);
					cmd_rot_2_pub.publish(rpy_gp_2);
					std::cout << "rpy_gp_1" << std::endl;
					std::cout << rpy_gp_1 << std::endl;
					std::cout << "rpy_gp_2" << std::endl;
					std::cout << rpy_gp_2 << std::endl;
					std::cout << "mu_min" << std::endl;
					std::cout << mu_min << std::endl;
					std::cout << "sigma_min" << std::endl;
					std::cout << sigma_min << std::endl;
					std::cout << "---" << std::endl;
					rpy_gp_update.data = 10.;
					for (int k = 0; k < 100; k++)
						BOsent_pub.publish(rpy_gp_update);
	    			}
	    			else if (!update_gp_done)
	    			{
	    				std::cout << "update non fatto" << std::endl;
	    			}
			}
			
		}
		
		BOsent_pub.publish(rpy_gp_update);
		
		ros::spinOnce();
	
	}
	
	return 0;

}
